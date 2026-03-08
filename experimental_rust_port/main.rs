//! # Smart Stables Monitor — Rust Port
//!
//! **Original**: stables_1_0_0 (Arduino C++ for ESP32)
//! **Author**: Maciej Kasperek (von Krappitz)
//! **Version**: 1.0.0 Stable (Rust port)
//! **Date**: 2026-02-22
//!
//! ## Architecture Notes
//!
//! This is a direct port of the Arduino `.ino` firmware to idiomatic Rust
//! using the `esp-idf-hal` / `esp-idf-svc` ecosystem. Key differences:
//!
//! - `RTC_DATA_ATTR` → `#[link_section = ".rtc.data"]` static muts
//! - Arduino `Wire` / `Serial` → `esp-idf-hal` I2C / UART drivers
//! - All sensor communication is done via raw I2C register operations
//!   (no external crate equivalents for Adafruit libs)
//! - Watchdog, deep-sleep, brownout — via `esp-idf-sys` raw bindings
//!
//! ## Hardware Bus Map (TCA9548A MUX @ 0x70)
//!
//! | Channel | Device        | I2C Addr |
//! |---------|---------------|----------|
//! | 0       | SSD1306 OLED  | 0x3C     |
//! | 1       | AHT20 T/H     | 0x38     |
//! | 2       | DS3231 RTC    | 0x68     |
//! | 3       | ADS1115 ADC   | 0x48     |
//! | 4       | Qwiic Joystick| 0x20     |

#![allow(static_mut_refs)]

use anyhow::{bail, Result};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{self, PinDriver, OutputPin};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::{UartDriver, UartConfig};
use esp_idf_sys as sys;
use log::{info, warn, error};
use std::fmt::Write as FmtWrite;

// =============================================================================
// CONSTANTS & CONFIGURATION
// =============================================================================

const FW_VERSION: &str = "1.0.0";

const NTP_SERVER: &str = "pool.ntp.org";
const GMT_OFFSET_SEC: i32 = 3600;
const DAYLIGHT_OFFSET_SEC: i32 = 3600;

// Timings (tuned for sensor chemical stability)
const WAKEUP_OFFSET_SEC: u64 = 200;
const FAN_DURATION_SEC: u64 = 15;
const SETTLE_DURATION_SEC: u64 = 105;
const PMS_WARMUP_SEC: u64 = 30;

// MOSFET power gate for MH-Z19
// GPIO13: P-MOSFET gate (IRLML6401) — LOW = ON, HIGH = OFF
const MHZ19_PREHEAT_SEC: u64 = 180;

// OLED display result time
const DISPLAY_RESULT_MS: u64 = 5000;

// Memory & file limits
const MAX_OFFLINE_RECORDS: usize = 10;
const MAX_LINE_LENGTH: usize = 120;
const MAX_FILE_SIZE_BYTES: u64 = 52_428_800; // 50MB FAT32 limit

// SD recovery interval
const SD_RECOVERY_INTERVAL: u32 = 10;

// Stuck joystick protection
const SVC_MODE_STREAK_MAX: u8 = 3;

// UI timeouts
const UI_TIMEOUT_MS: u64 = 60_000;

const DEFAULT_NH3_FACTOR: f32 = 0.0025;

const CSV_HEADER: &str = "Timestamp,Temp_C,Humidity_%,NH3_Raw,NH3_PPM,CO2_ppm,PM1,PM2.5,PM10";

// I2C addresses
const MUX_ADDR: u8 = 0x70;
const OLED_ADDR: u8 = 0x3C;
const AHT_ADDR: u8 = 0x38;
const RTC_ADDR: u8 = 0x68;
const ADS_ADDR: u8 = 0x48;
const JOY_ADDR: u8 = 0x20;

// MUX channels
const CH_OLED: u8 = 0;
const CH_AHT: u8 = 1;
const CH_RTC: u8 = 2;
const CH_ADS: u8 = 3;
const CH_JOY: u8 = 4;

// GPIO Pins (references, actual binding in main)
const PIN_FAN: i32 = 4;
const PIN_CS_SD: i32 = 5;
const PIN_PMS_RX: i32 = 16;
const PIN_PMS_TX: i32 = 17;
const PIN_CO2_RX: i32 = 32;
const PIN_CO2_TX: i32 = 33;
const PIN_MOSFET: i32 = 13;

// =============================================================================
// RTC-BACKED VARIABLES  (survive deep sleep)
// =============================================================================
//
// In ESP-IDF Rust these go into `.rtc.data` section.  
// SAFETY: accessed only from the single-threaded main context (no ISR, no RTOS task).

#[link_section = ".rtc.data"]
static mut RTC_BUFFER: [[u8; MAX_LINE_LENGTH]; MAX_OFFLINE_RECORDS] =
    [[0u8; MAX_LINE_LENGTH]; MAX_OFFLINE_RECORDS];

#[link_section = ".rtc.data"]
static mut BUFFER_HEAD: i32 = 0;
#[link_section = ".rtc.data"]
static mut BUFFER_TAIL: i32 = 0;
#[link_section = ".rtc.data"]
static mut RTC_BUFFER_COUNT: i32 = 0;
#[link_section = ".rtc.data"]
static mut SD_FAIL_COUNT: i32 = 0;

#[link_section = ".rtc.data"]
static mut CURRENT_FILE_SIZE: u64 = 0;
#[link_section = ".rtc.data"]
static mut SIZE_SYNCED: bool = false;
#[link_section = ".rtc.data"]
static mut WRITES_SINCE_SYNC: i32 = 0;

#[link_section = ".rtc.data"]
static mut LAST_FILE_NAME: [u8; 35] = [0u8; 35];

#[link_section = ".rtc.data"]
static mut WAKE_COUNT: u32 = 0;

#[link_section = ".rtc.data"]
static mut SD_RECOVERY_CYCLE: u32 = 0;

#[link_section = ".rtc.data"]
static mut SVC_MODE_STREAK: u8 = 0;

#[link_section = ".rtc.data"]
static mut BROWNOUT_REG_SAVED: u32 = 0;

// =============================================================================
// RUNTIME STATE (cleared on every boot)
// =============================================================================

struct RuntimeState {
    ads_initialized: bool,
    joy_initialized: bool,
    display_ready: bool,
    pms_uart_ready: bool,
    mux_err_count: u16,
    config_ssid: String,
    config_pass: String,
    nh3_baseline: i32,
    nh3_factor: f32,
}

impl Default for RuntimeState {
    fn default() -> Self {
        Self {
            ads_initialized: false,
            joy_initialized: false,
            display_ready: false,
            pms_uart_ready: false,
            mux_err_count: 0,
            config_ssid: String::new(),
            config_pass: String::new(),
            nh3_baseline: 0,
            nh3_factor: DEFAULT_NH3_FACTOR,
        }
    }
}

// =============================================================================
// I2C MUX (TCA9548A) DRIVER
// =============================================================================

/// Select a channel on the TCA9548A I2C multiplexer.
/// Returns `true` on success, `false` on bus error.
fn tca_select(i2c: &mut I2cDriver<'_>, channel: u8, state: &mut RuntimeState) -> bool {
    if channel > 7 {
        return false;
    }
    let data = [1u8 << channel];
    match i2c.write(MUX_ADDR, &data, 100) {
        Ok(()) => true,
        Err(e) => {
            state.mux_err_count += 1;
            error!("[{}] MUX ERR ch={} {:?}", FW_VERSION, channel, e);
            false
        }
    }
}

// =============================================================================
// SSD1306 OLED DRIVER  (128×64, I2C)
// =============================================================================

/// Minimal SSD1306 driver — command/data over I2C.
struct Oled;

impl Oled {
    const WIDTH: usize = 128;
    const HEIGHT: usize = 64;
    const PAGES: usize = Self::HEIGHT / 8;

    /// Initialise the display (equivalent of `display.begin(SSD1306_SWITCHCAPVCC, 0x3C)`).
    fn init(i2c: &mut I2cDriver<'_>) -> Result<()> {
        let init_cmds: &[u8] = &[
            0xAE, // Display OFF
            0xD5, 0x80, // Set display clock
            0xA8, 0x3F, // Set multiplex ratio (64-1)
            0xD3, 0x00, // Display offset = 0
            0x40,       // Start line = 0
            0x8D, 0x14, // Charge pump ON (SWITCHCAPVCC)
            0x20, 0x00, // Horizontal addressing mode
            0xA1,       // Segment re-map
            0xC8,       // COM output scan direction
            0xDA, 0x12, // COM pins config
            0x81, 0xCF, // Contrast
            0xD9, 0xF1, // Pre-charge
            0xDB, 0x40, // VCOMH deselect
            0xA4,       // Display from RAM
            0xA6,       // Normal display (not inverted)
            0xAF,       // Display ON
        ];
        for &cmd in init_cmds {
            i2c.write(OLED_ADDR, &[0x00, cmd], 100)?;
        }
        Ok(())
    }

    /// Send the entire framebuffer (1024 bytes for 128×64 mono).
    fn flush(i2c: &mut I2cDriver<'_>, fb: &[u8; 1024]) -> Result<()> {
        // Set column & page address to full range
        for &cmd in &[0x21, 0x00, 0x7F, 0x22, 0x00, 0x07] {
            i2c.write(OLED_ADDR, &[0x00, cmd], 100)?;
        }
        // Write data in 128-byte chunks (I2C buffer limit)
        for page in 0..Self::PAGES {
            let start = page * Self::WIDTH;
            let mut buf = [0u8; 129]; // 1 control byte + 128 data
            buf[0] = 0x40; // Data mode
            buf[1..].copy_from_slice(&fb[start..start + Self::WIDTH]);
            i2c.write(OLED_ADDR, &buf, 100)?;
        }
        Ok(())
    }

    /// Turn display off (save power during deep sleep).
    fn display_off(i2c: &mut I2cDriver<'_>) -> Result<()> {
        i2c.write(OLED_ADDR, &[0x00, 0xAE], 100)?;
        Ok(())
    }

    /// Clear entire framebuffer.
    fn clear(fb: &mut [u8; 1024]) {
        fb.fill(0);
    }
}

// =============================================================================
// STANDARD 5×7 FONT TABLE  (ASCII 0x20–0x7E, column-major, LSB=top)
// =============================================================================

/// Adafruit GFX compatible 5×7 font. 95 glyphs × 5 bytes each.
/// Each byte is one column, bit 0 = top pixel, bit 6 = bottom pixel.
#[rustfmt::skip]
const FONT_5X7: [[u8; 5]; 95] = [
    [0x00,0x00,0x00,0x00,0x00], // 0x20 ' '
    [0x00,0x00,0x5F,0x00,0x00], // 0x21 '!'
    [0x00,0x07,0x00,0x07,0x00], // 0x22 '"'
    [0x14,0x7F,0x14,0x7F,0x14], // 0x23 '#'
    [0x24,0x2A,0x7F,0x2A,0x12], // 0x24 '$'
    [0x23,0x13,0x08,0x64,0x62], // 0x25 '%'
    [0x36,0x49,0x55,0x22,0x50], // 0x26 '&'
    [0x00,0x05,0x03,0x00,0x00], // 0x27 '''
    [0x00,0x1C,0x22,0x41,0x00], // 0x28 '('
    [0x00,0x41,0x22,0x1C,0x00], // 0x29 ')'
    [0x14,0x08,0x3E,0x08,0x14], // 0x2A '*'
    [0x08,0x08,0x3E,0x08,0x08], // 0x2B '+'
    [0x00,0x50,0x30,0x00,0x00], // 0x2C ','
    [0x08,0x08,0x08,0x08,0x08], // 0x2D '-'
    [0x00,0x60,0x60,0x00,0x00], // 0x2E '.'
    [0x20,0x10,0x08,0x04,0x02], // 0x2F '/'
    [0x3E,0x51,0x49,0x45,0x3E], // 0x30 '0'
    [0x00,0x42,0x7F,0x40,0x00], // 0x31 '1'
    [0x42,0x61,0x51,0x49,0x46], // 0x32 '2'
    [0x21,0x41,0x45,0x4B,0x31], // 0x33 '3'
    [0x18,0x14,0x12,0x7F,0x10], // 0x34 '4'
    [0x27,0x45,0x45,0x45,0x39], // 0x35 '5'
    [0x3C,0x4A,0x49,0x49,0x30], // 0x36 '6'
    [0x01,0x71,0x09,0x05,0x03], // 0x37 '7'
    [0x36,0x49,0x49,0x49,0x36], // 0x38 '8'
    [0x06,0x49,0x49,0x29,0x1E], // 0x39 '9'
    [0x00,0x36,0x36,0x00,0x00], // 0x3A ':'
    [0x00,0x56,0x36,0x00,0x00], // 0x3B ';'
    [0x08,0x14,0x22,0x41,0x00], // 0x3C '<'
    [0x14,0x14,0x14,0x14,0x14], // 0x3D '='
    [0x00,0x41,0x22,0x14,0x08], // 0x3E '>'
    [0x02,0x01,0x51,0x09,0x06], // 0x3F '?'
    [0x32,0x49,0x79,0x41,0x3E], // 0x40 '@'
    [0x7E,0x11,0x11,0x11,0x7E], // 0x41 'A'
    [0x7F,0x49,0x49,0x49,0x36], // 0x42 'B'
    [0x3E,0x41,0x41,0x41,0x22], // 0x43 'C'
    [0x7F,0x41,0x41,0x22,0x1C], // 0x44 'D'
    [0x7F,0x49,0x49,0x49,0x41], // 0x45 'E'
    [0x7F,0x09,0x09,0x09,0x01], // 0x46 'F'
    [0x3E,0x41,0x49,0x49,0x7A], // 0x47 'G'
    [0x7F,0x08,0x08,0x08,0x7F], // 0x48 'H'
    [0x00,0x41,0x7F,0x41,0x00], // 0x49 'I'
    [0x20,0x40,0x41,0x3F,0x01], // 0x4A 'J'
    [0x7F,0x08,0x14,0x22,0x41], // 0x4B 'K'
    [0x7F,0x40,0x40,0x40,0x40], // 0x4C 'L'
    [0x7F,0x02,0x0C,0x02,0x7F], // 0x4D 'M'
    [0x7F,0x04,0x08,0x10,0x7F], // 0x4E 'N'
    [0x3E,0x41,0x41,0x41,0x3E], // 0x4F 'O'
    [0x7F,0x09,0x09,0x09,0x06], // 0x50 'P'
    [0x3E,0x41,0x51,0x21,0x5E], // 0x51 'Q'
    [0x7F,0x09,0x19,0x29,0x46], // 0x52 'R'
    [0x46,0x49,0x49,0x49,0x31], // 0x53 'S'
    [0x01,0x01,0x7F,0x01,0x01], // 0x54 'T'
    [0x3F,0x40,0x40,0x40,0x3F], // 0x55 'U'
    [0x1F,0x20,0x40,0x20,0x1F], // 0x56 'V'
    [0x3F,0x40,0x38,0x40,0x3F], // 0x57 'W'
    [0x63,0x14,0x08,0x14,0x63], // 0x58 'X'
    [0x07,0x08,0x70,0x08,0x07], // 0x59 'Y'
    [0x61,0x51,0x49,0x45,0x43], // 0x5A 'Z'
    [0x00,0x7F,0x41,0x41,0x00], // 0x5B '['
    [0x02,0x04,0x08,0x10,0x20], // 0x5C '\'
    [0x00,0x41,0x41,0x7F,0x00], // 0x5D ']'
    [0x04,0x02,0x01,0x02,0x04], // 0x5E '^'
    [0x40,0x40,0x40,0x40,0x40], // 0x5F '_'
    [0x00,0x01,0x02,0x04,0x00], // 0x60 '`'
    [0x20,0x54,0x54,0x54,0x78], // 0x61 'a'
    [0x7F,0x48,0x44,0x44,0x38], // 0x62 'b'
    [0x38,0x44,0x44,0x44,0x20], // 0x63 'c'
    [0x38,0x44,0x44,0x48,0x7F], // 0x64 'd'
    [0x38,0x54,0x54,0x54,0x18], // 0x65 'e'
    [0x08,0x7E,0x09,0x01,0x02], // 0x66 'f'
    [0x0C,0x52,0x52,0x52,0x3E], // 0x67 'g'
    [0x7F,0x08,0x04,0x04,0x78], // 0x68 'h'
    [0x00,0x44,0x7D,0x40,0x00], // 0x69 'i'
    [0x20,0x40,0x44,0x3D,0x00], // 0x6A 'j'
    [0x7F,0x10,0x28,0x44,0x00], // 0x6B 'k'
    [0x00,0x41,0x7F,0x40,0x00], // 0x6C 'l'
    [0x7C,0x04,0x18,0x04,0x78], // 0x6D 'm'
    [0x7C,0x08,0x04,0x04,0x78], // 0x6E 'n'
    [0x38,0x44,0x44,0x44,0x38], // 0x6F 'o'
    [0x7C,0x14,0x14,0x14,0x08], // 0x70 'p'
    [0x08,0x14,0x14,0x18,0x7C], // 0x71 'q'
    [0x7C,0x08,0x04,0x04,0x08], // 0x72 'r'
    [0x48,0x54,0x54,0x54,0x20], // 0x73 's'
    [0x04,0x3F,0x44,0x40,0x20], // 0x74 't'
    [0x3C,0x40,0x40,0x20,0x7C], // 0x75 'u'
    [0x1C,0x20,0x40,0x20,0x1C], // 0x76 'v'
    [0x3C,0x40,0x30,0x40,0x3C], // 0x77 'w'
    [0x44,0x28,0x10,0x28,0x44], // 0x78 'x'
    [0x0C,0x50,0x50,0x50,0x3C], // 0x79 'y'
    [0x44,0x64,0x54,0x4C,0x44], // 0x7A 'z'
    [0x00,0x08,0x36,0x41,0x00], // 0x7B '{'
    [0x00,0x00,0x7F,0x00,0x00], // 0x7C '|'
    [0x00,0x41,0x36,0x08,0x00], // 0x7D '}'
    [0x10,0x08,0x08,0x10,0x08], // 0x7E '~'
];

// =============================================================================
// TEXT RENDERER + FRAMEBUFFER DRAWING  (6×8 fixed-width font)
// =============================================================================

/// 6×8 font renderer for SSD1306 framebuffer (128×64, page-addressed).
struct TextRenderer {
    cursor_x: usize,
    cursor_y: usize,
}

impl TextRenderer {
    fn new() -> Self {
        Self { cursor_x: 0, cursor_y: 0 }
    }

    fn set_cursor(&mut self, x: usize, y: usize) {
        self.cursor_x = x;
        self.cursor_y = y;
    }

    /// Look up a glyph from the font table.
    fn glyph(ch: char) -> [u8; 5] {
        let c = ch as u8;
        if c >= 0x20 && c <= 0x7E {
            FONT_5X7[(c - 0x20) as usize]
        } else {
            [0x7F, 0x41, 0x41, 0x41, 0x7F] // box for unknowns
        }
    }

    /// Set a single pixel in the framebuffer (x, y coords).
    fn set_pixel(fb: &mut [u8; 1024], x: usize, y: usize, on: bool) {
        if x >= Oled::WIDTH || y >= Oled::HEIGHT {
            return;
        }
        let page = y / 8;
        let bit = y % 8;
        let idx = page * Oled::WIDTH + x;
        if on {
            fb[idx] |= 1 << bit;
        } else {
            fb[idx] &= !(1 << bit);
        }
    }

    /// Draw a filled rectangle in the framebuffer.
    fn fill_rect(fb: &mut [u8; 1024], x: usize, y: usize, w: usize, h: usize, on: bool) {
        for dy in 0..h {
            for dx in 0..w {
                Self::set_pixel(fb, x + dx, y + dy, on);
            }
        }
    }

    /// Print a string, normal (white on black). Newlines advance by 8 px.
    fn print(&mut self, fb: &mut [u8; 1024], text: &str) {
        for ch in text.chars() {
            if ch == '\n' {
                self.cursor_x = 0;
                self.cursor_y += 8;
                continue;
            }
            if self.cursor_x + 6 > Oled::WIDTH || self.cursor_y >= Oled::HEIGHT {
                continue;
            }
            let glyph = Self::glyph(ch);
            let page = self.cursor_y / 8;
            let bit_offset = self.cursor_y % 8;
            if bit_offset == 0 {
                // Fast path: page-aligned
                for col in 0..5 {
                    let idx = page * Oled::WIDTH + self.cursor_x + col;
                    if idx < 1024 {
                        fb[idx] |= glyph[col];
                    }
                }
            } else {
                // Slow path: spans two pages
                for col in 0..5 {
                    let idx0 = page * Oled::WIDTH + self.cursor_x + col;
                    if idx0 < 1024 {
                        fb[idx0] |= glyph[col] << bit_offset;
                    }
                    let idx1 = (page + 1) * Oled::WIDTH + self.cursor_x + col;
                    if idx1 < 1024 {
                        fb[idx1] |= glyph[col] >> (8 - bit_offset);
                    }
                }
            }
            self.cursor_x += 6;
        }
    }

    /// Print with inverted colors (black text on white background).
    fn print_inverted(&mut self, fb: &mut [u8; 1024], text: &str) {
        let x0 = self.cursor_x;
        let y0 = self.cursor_y;
        let w = text.len() * 6;
        Self::fill_rect(fb, x0, y0, w + 2, 8, true);
        // Draw text by inverting — write glyph as black pixels on the white rect
        for ch in text.chars() {
            if ch == '\n' {
                self.cursor_x = 0;
                self.cursor_y += 8;
                continue;
            }
            if self.cursor_x + 6 > Oled::WIDTH || self.cursor_y >= Oled::HEIGHT {
                continue;
            }
            let glyph = Self::glyph(ch);
            let page = self.cursor_y / 8;
            let bit_offset = self.cursor_y % 8;
            if bit_offset == 0 {
                for col in 0..5 {
                    let idx = page * Oled::WIDTH + self.cursor_x + col;
                    if idx < 1024 {
                        fb[idx] &= !glyph[col]; // clear glyph bits = black text
                    }
                }
            } else {
                for col in 0..5 {
                    let idx0 = page * Oled::WIDTH + self.cursor_x + col;
                    if idx0 < 1024 {
                        fb[idx0] &= !(glyph[col] << bit_offset);
                    }
                    let idx1 = (page + 1) * Oled::WIDTH + self.cursor_x + col;
                    if idx1 < 1024 {
                        fb[idx1] &= !(glyph[col] >> (8 - bit_offset));
                    }
                }
            }
            self.cursor_x += 6;
        }
    }

    fn println(&mut self, fb: &mut [u8; 1024], text: &str) {
        self.print(fb, text);
        self.cursor_x = 0;
        self.cursor_y += 8;
    }

    /// Draw a horizontal progress bar.
    fn progress_bar(fb: &mut [u8; 1024], x: usize, y: usize, w_max: usize, h: usize, fraction: f32) {
        let filled = ((w_max as f32) * fraction.clamp(0.0, 1.0)) as usize;
        Self::fill_rect(fb, x, y, filled, h, true);
    }
}

// =============================================================================
// AHT20 TEMPERATURE / HUMIDITY SENSOR DRIVER
// =============================================================================

struct Aht20;

impl Aht20 {
    /// Trigger a measurement and read temperature + humidity.
    fn read(i2c: &mut I2cDriver<'_>) -> Result<(f32, f32)> {
        // Trigger measurement: command 0xAC with data bytes 0x33, 0x00
        i2c.write(AHT_ADDR, &[0xAC, 0x33, 0x00], 100)?;
        FreeRtos::delay_ms(80); // Measurement takes ~75ms

        let mut buf = [0u8; 7];
        i2c.read(AHT_ADDR, &mut buf, 100)?;

        // Check busy bit
        if buf[0] & 0x80 != 0 {
            bail!("AHT20 still busy");
        }

        // Parse 20-bit humidity and 20-bit temperature
        let raw_hum = ((buf[1] as u32) << 12) | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4);
        let raw_temp = (((buf[3] & 0x0F) as u32) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32);

        let humidity = (raw_hum as f32 / 1_048_576.0) * 100.0;
        let temperature = (raw_temp as f32 / 1_048_576.0) * 200.0 - 50.0;

        Ok((temperature, humidity))
    }
}

// =============================================================================
// DS3231 RTC DRIVER
// =============================================================================

#[derive(Debug, Clone, Copy)]
struct DateTime {
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
}

impl DateTime {
    fn timestamp_str(&self) -> String {
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
            self.year, self.month, self.day, self.hour, self.minute, self.second
        )
    }

    fn filename(&self) -> String {
        format!("/pomiar_{:04}-{:02}-{:02}.csv", self.year, self.month, self.day)
    }
}

struct Ds3231;

impl Ds3231 {
    fn bcd_to_dec(bcd: u8) -> u8 {
        (bcd >> 4) * 10 + (bcd & 0x0F)
    }

    fn dec_to_bcd(dec: u8) -> u8 {
        ((dec / 10) << 4) | (dec % 10)
    }

    fn read_time(i2c: &mut I2cDriver<'_>) -> Result<DateTime> {
        i2c.write(RTC_ADDR, &[0x00], 100)?;
        let mut buf = [0u8; 7];
        i2c.read(RTC_ADDR, &mut buf, 100)?;

        Ok(DateTime {
            second: Self::bcd_to_dec(buf[0] & 0x7F),
            minute: Self::bcd_to_dec(buf[1] & 0x7F),
            hour: Self::bcd_to_dec(buf[2] & 0x3F),
            day: Self::bcd_to_dec(buf[4] & 0x3F),
            month: Self::bcd_to_dec(buf[5] & 0x1F),
            year: 2000 + Self::bcd_to_dec(buf[6]) as u16,
        })
    }

    fn set_time(i2c: &mut I2cDriver<'_>, dt: &DateTime) -> Result<()> {
        let data = [
            0x00, // register address
            Self::dec_to_bcd(dt.second),
            Self::dec_to_bcd(dt.minute),
            Self::dec_to_bcd(dt.hour),
            0x01, // day-of-week (unused)
            Self::dec_to_bcd(dt.day),
            Self::dec_to_bcd(dt.month),
            Self::dec_to_bcd((dt.year - 2000) as u8),
        ];
        i2c.write(RTC_ADDR, &data, 100)?;
        Ok(())
    }
}

// =============================================================================
// ADS1115 16-BIT ADC DRIVER
// =============================================================================

struct Ads1115;

impl Ads1115 {
    /// Read single-ended channel 0 with PGA = ±4.096V (GAIN_ONE).
    fn read_single_ended_ch0(i2c: &mut I2cDriver<'_>) -> Result<i16> {
        // Config register: single-shot, AIN0 vs GND, ±4.096V, 128SPS
        let config: u16 = 0xC383; // OS=1, MUX=100, PGA=001, MODE=1, DR=100, rest defaults
        let cfg_bytes = config.to_be_bytes();
        i2c.write(ADS_ADDR, &[0x01, cfg_bytes[0], cfg_bytes[1]], 100)?;

        // Wait for conversion (~8ms at 128SPS)
        FreeRtos::delay_ms(10);

        // Poll config register for conversion complete (bit 15)
        for _ in 0..10 {
            i2c.write(ADS_ADDR, &[0x01], 100)?;
            let mut status = [0u8; 2];
            i2c.read(ADS_ADDR, &mut status, 100)?;
            if status[0] & 0x80 != 0 {
                break;
            }
            FreeRtos::delay_ms(2);
        }

        // Read conversion register
        i2c.write(ADS_ADDR, &[0x00], 100)?;
        let mut result = [0u8; 2];
        i2c.read(ADS_ADDR, &mut result, 100)?;

        Ok(i16::from_be_bytes(result))
    }
}

// =============================================================================
// QWIIC JOYSTICK DRIVER  (SparkFun, I2C addr 0x20)
// =============================================================================

struct QwiicJoystick;

impl QwiicJoystick {
    /// Read horizontal axis (0–1023, center ~512).
    fn horizontal(i2c: &mut I2cDriver<'_>) -> Result<u16> {
        i2c.write(JOY_ADDR, &[0x03], 100)?;
        let mut buf = [0u8; 2];
        i2c.read(JOY_ADDR, &mut buf, 100)?;
        Ok(((buf[0] as u16) << 8) | buf[1] as u16)
    }

    /// Read vertical axis (0–1023, center ~512).
    fn vertical(i2c: &mut I2cDriver<'_>) -> Result<u16> {
        i2c.write(JOY_ADDR, &[0x05], 100)?;
        let mut buf = [0u8; 2];
        i2c.read(JOY_ADDR, &mut buf, 100)?;
        Ok(((buf[0] as u16) << 8) | buf[1] as u16)
    }

    /// Read button state: 0 = pressed, 1 = released.
    fn button(i2c: &mut I2cDriver<'_>) -> Result<u8> {
        i2c.write(JOY_ADDR, &[0x07], 100)?;
        let mut buf = [0u8; 1];
        i2c.read(JOY_ADDR, &mut buf, 100)?;
        Ok(buf[0])
    }

    /// Check if joystick is present on the bus.
    fn begin(i2c: &mut I2cDriver<'_>) -> bool {
        i2c.write(JOY_ADDR, &[0x00], 100).is_ok()
    }
}

// =============================================================================
// MH-Z19 CO2 SENSOR DRIVER  (UART, 9600 baud)
// =============================================================================

struct Mhz19;

impl Mhz19 {
    /// Read CO2 concentration in ppm. Returns `None` on failure.
    fn read_co2(uart: &mut UartDriver<'_>) -> Option<i32> {
        // Send read command: FF 01 86 00 00 00 00 00 79
        let cmd: [u8; 9] = [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];
        // Flush RX buffer
        let mut trash = [0u8; 64];
        let _ = uart.read(&mut trash, 0);

        if uart.write(&cmd).is_err() {
            return None;
        }

        FreeRtos::delay_ms(200);

        let mut resp = [0u8; 9];
        let mut total = 0usize;
        for _ in 0..20 {
            match uart.read(&mut resp[total..], 50) {
                Ok(n) if n > 0 => {
                    total += n;
                    if total >= 9 {
                        break;
                    }
                }
                _ => FreeRtos::delay_ms(10),
            }
        }

        if total < 9 || resp[0] != 0xFF || resp[1] != 0x86 {
            return None;
        }

        // Checksum
        let mut csum: u8 = 0;
        for &b in &resp[1..8] {
            csum = csum.wrapping_add(b);
        }
        csum = (!csum).wrapping_add(1);
        if csum != resp[8] {
            return None;
        }

        let ppm = (resp[2] as i32) * 256 + (resp[3] as i32);
        if ppm > 0 && ppm <= 10000 {
            Some(ppm)
        } else {
            None
        }
    }

    /// Disable auto-calibration (ABC logic).
    fn auto_calibration_off(uart: &mut UartDriver<'_>) {
        let cmd: [u8; 9] = [0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86];
        let _ = uart.write(&cmd);
    }

    /// Zero-point (400 ppm) calibration.
    fn calibrate(uart: &mut UartDriver<'_>) {
        let cmd: [u8; 9] = [0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78];
        let _ = uart.write(&cmd);
    }
}

// =============================================================================
// PMS5003 PARTICULATE MATTER SENSOR DRIVER  (UART, 9600 baud)
// =============================================================================

#[derive(Debug, Default, Clone, Copy)]
struct PmsData {
    pm1_0: u16,
    pm2_5: u16,
    pm10: u16,
}

struct Pms5003;

impl Pms5003 {
    /// Wake the sensor from sleep mode.
    fn wake(uart: &mut UartDriver<'_>) {
        let cmd: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74];
        let _ = uart.write(&cmd);
    }

    /// Put the sensor to sleep.
    fn sleep(uart: &mut UartDriver<'_>) {
        let cmd: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73];
        let _ = uart.write(&cmd);
    }

    /// Read a data frame. Timeout in ms.
    fn read(uart: &mut UartDriver<'_>, timeout_ms: u64) -> Option<PmsData> {
        let start = Self::millis();
        let mut buf = [0u8; 32];
        let mut idx = 0usize;

        loop {
            if Self::millis() - start > timeout_ms {
                return None;
            }
            let mut byte = [0u8; 1];
            match uart.read(&mut byte, 10) {
                Ok(1) => {
                    if idx == 0 && byte[0] != 0x42 {
                        continue;
                    }
                    if idx == 1 && byte[0] != 0x4D {
                        idx = 0;
                        continue;
                    }
                    buf[idx] = byte[0];
                    idx += 1;
                    if idx >= 32 {
                        break;
                    }
                }
                _ => continue,
            }
        }

        // Verify checksum
        let check = u16::from_be_bytes([buf[30], buf[31]]);
        let mut sum = 0u16;
        for &b in &buf[..30] {
            sum = sum.wrapping_add(b as u16);
        }
        if check != sum {
            return None;
        }

        // Atmospheric environment readings (bytes 10-15)
        Some(PmsData {
            pm1_0: u16::from_be_bytes([buf[10], buf[11]]),
            pm2_5: u16::from_be_bytes([buf[12], buf[13]]),
            pm10: u16::from_be_bytes([buf[14], buf[15]]),
        })
    }

    fn millis() -> u64 {
        unsafe { sys::esp_timer_get_time() as u64 / 1000 }
    }
}

// =============================================================================
// SD CARD (FAT via esp-idf VFS)
// =============================================================================

mod sdcard {
    use super::*;

    /// Mount the SD card via SPI using ESP-IDF's FATFS VFS layer.
    /// Returns `true` on success.
    ///
    /// NOTE: This uses raw `esp-idf-sys` calls because `esp-idf-hal` does not
    /// expose a high-level SD/MMC API. The mount point is "/sdcard".
    pub fn mount() -> bool {
        unsafe {
            let mount_cfg = sys::esp_vfs_fat_sdmmc_mount_config_t {
                format_if_mount_failed: false,
                max_files: 5,
                allocation_unit_size: 0,
                disk_status_check_enable: false,
            };

            let mut card: *mut sys::sdmmc_card_t = std::ptr::null_mut();

            let host = sys::sdmmc_host_t {
                flags: sys::SDMMC_HOST_FLAG_SPI,
                slot: 1, // HSPI
                max_freq_khz: 4000,
                io_voltage: 3.3,
                init: Some(sys::sdspi_host_init),
                set_bus_width: None,
                get_bus_width: None,
                set_bus_ddr_mode: None,
                set_card_clk: Some(sys::sdspi_host_set_card_clk),
                set_cclk_always_on: None,
                do_transaction: Some(sys::sdspi_host_do_transaction),
                __bindgen_anon_1: sys::sdmmc_host_t__bindgen_ty_1 {
                    deinit: Some(sys::sdspi_host_deinit),
                },
                io_int_enable: Some(sys::sdspi_host_io_int_enable),
                io_int_wait: Some(sys::sdspi_host_io_int_wait),
                command_timeout_ms: 0,
                get_real_freq: Some(sys::sdspi_host_get_real_freq),
                input_delay_phase: sys::sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_0,
                set_input_delay: None,
            };

            let slot_cfg = sys::sdspi_device_config_t {
                host_id: sys::spi_host_device_t_SPI2_HOST,
                gpio_cs: PIN_CS_SD,
                gpio_cd: -1,
                gpio_wp: -1,
                gpio_int: -1,
                gpio_wp_polarity: false,
            };

            let mount_point = std::ffi::CString::new("/sdcard").unwrap();

            let ret = sys::esp_vfs_fat_sdspi_mount(
                mount_point.as_ptr(),
                &host,
                &slot_cfg,
                &mount_cfg,
                &mut card,
            );

            ret == sys::ESP_OK as i32
        }
    }

    pub fn unmount() {
        unsafe {
            let mount_point = std::ffi::CString::new("/sdcard").unwrap();
            let _ = sys::esp_vfs_fat_sdcard_unmount(mount_point.as_ptr(), std::ptr::null_mut());
        }
    }

    /// Append a line to a file on the SD card. Creates file + header if needed.
    pub fn write_line(filename: &str, line: &str, file_size_cache: &mut u64) -> bool {
        let path = format!("/sdcard{}", filename);
        let c_path = match std::ffi::CString::new(path.as_str()) {
            Ok(p) => p,
            Err(_) => return false,
        };

        // Try append first, then write
        let mode_a = std::ffi::CString::new("a").unwrap();
        let mode_r = std::ffi::CString::new("r").unwrap();

        unsafe {
            let f = libc::fopen(c_path.as_ptr(), mode_a.as_ptr());
            if f.is_null() {
                return false;
            }

            // Check if file is new (empty)
            libc::fseek(f, 0, libc::SEEK_END);
            let size = libc::ftell(f) as u64;

            if size == 0 {
                let header = std::ffi::CString::new(format!("{}\n", CSV_HEADER)).unwrap();
                libc::fputs(header.as_ptr(), f);
            }

            if size > MAX_FILE_SIZE_BYTES {
                libc::fclose(f);
                return false;
            }

            let data = std::ffi::CString::new(format!("{}\n", line)).unwrap();
            let written = libc::fputs(data.as_ptr(), f);
            libc::fclose(f);

            if written >= 0 {
                *file_size_cache = size + line.len() as u64 + 1;
                true
            } else {
                false
            }
        }
    }
}

// =============================================================================
// NVS PREFERENCES (equivalent of Arduino Preferences)
// =============================================================================

mod preferences {
    use esp_idf_svc::nvs::*;
    use esp_idf_svc::nvs_storage::EspNvs;
    use anyhow::Result;

    pub fn get_string(namespace: &str, key: &str, default: &str) -> String {
        let nvs_partition = EspNvsPartition::<NvsDefault>::take();
        if let Ok(part) = nvs_partition {
            if let Ok(nvs) = EspNvs::new(part, namespace, true) {
                let mut buf = [0u8; 128];
                if let Ok(Some(val)) = nvs.get_str(key, &mut buf) {
                    return val.to_string();
                }
            }
        }
        default.to_string()
    }

    pub fn put_string(namespace: &str, key: &str, value: &str) -> Result<()> {
        let part = EspNvsPartition::<NvsDefault>::take()?;
        let mut nvs = EspNvs::new(part, namespace, false)?;
        nvs.set_str(key, value)?;
        Ok(())
    }

    pub fn get_i32(namespace: &str, key: &str, default: i32) -> i32 {
        let nvs_partition = EspNvsPartition::<NvsDefault>::take();
        if let Ok(part) = nvs_partition {
            if let Ok(nvs) = EspNvs::new(part, namespace, true) {
                if let Ok(Some(val)) = nvs.get_i32(key) {
                    return val;
                }
            }
        }
        default
    }

    pub fn put_i32(namespace: &str, key: &str, value: i32) -> Result<()> {
        let part = EspNvsPartition::<NvsDefault>::take()?;
        let mut nvs = EspNvs::new(part, namespace, false)?;
        nvs.set_i32(key, value)?;
        Ok(())
    }

    /// NVS does not natively support f32, so we store as raw bytes.
    pub fn get_f32(namespace: &str, key: &str, default: f32) -> f32 {
        let nvs_partition = EspNvsPartition::<NvsDefault>::take();
        if let Ok(part) = nvs_partition {
            if let Ok(nvs) = EspNvs::new(part, namespace, true) {
                let mut buf = [0u8; 4];
                if let Ok(Some(_)) = nvs.get_raw(key, &mut buf) {
                    return f32::from_le_bytes(buf);
                }
            }
        }
        default
    }

    pub fn put_f32(namespace: &str, key: &str, value: f32) -> Result<()> {
        let part = EspNvsPartition::<NvsDefault>::take()?;
        let mut nvs = EspNvs::new(part, namespace, false)?;
        nvs.set_raw(key, &value.to_le_bytes())?;
        Ok(())
    }
}

// =============================================================================
// BROWNOUT CONTROL
// =============================================================================

mod brownout {
    use esp_idf_sys as sys;
    use super::BROWNOUT_REG_SAVED;

    const RTC_CNTL_BROWN_OUT_REG: u32 = 0x3FF4_80D4;
    const RTC_CNTL_BROWN_OUT_ENA: u32 = 1 << 30;

    pub fn save_once() {
        unsafe {
            if BROWNOUT_REG_SAVED == 0 {
                BROWNOUT_REG_SAVED = core::ptr::read_volatile(RTC_CNTL_BROWN_OUT_REG as *const u32);
            }
        }
    }

    pub fn disable() {
        save_once();
        unsafe {
            let reg = RTC_CNTL_BROWN_OUT_REG as *mut u32;
            let val = core::ptr::read_volatile(reg);
            core::ptr::write_volatile(reg, val & !RTC_CNTL_BROWN_OUT_ENA);
        }
    }

    pub fn enable() {
        unsafe {
            let reg = RTC_CNTL_BROWN_OUT_REG as *mut u32;
            if BROWNOUT_REG_SAVED != 0 {
                core::ptr::write_volatile(reg, BROWNOUT_REG_SAVED);
            } else {
                let val = core::ptr::read_volatile(reg);
                core::ptr::write_volatile(reg, val | RTC_CNTL_BROWN_OUT_ENA);
            }
        }
    }
}

// =============================================================================
// WATCHDOG
// =============================================================================

fn init_watchdog() {
    unsafe {
        let cfg = sys::esp_task_wdt_config_t {
            timeout_ms: 45_000,
            idle_core_mask: 1 << 0,
            trigger_panic: true,
        };
        sys::esp_task_wdt_init(&cfg);
        sys::esp_task_wdt_add(std::ptr::null_mut());
    }
}

fn feed_watchdog() {
    unsafe {
        sys::esp_task_wdt_reset();
    }
}

// =============================================================================
// DEEP SLEEP
// =============================================================================

fn deep_sleep_us(us: u64) -> ! {
    unsafe {
        sys::esp_sleep_enable_timer_wakeup(us);
        sys::esp_deep_sleep_start();
    }
    // never returns
    unreachable!()
}

// =============================================================================
// MILLIS HELPER
// =============================================================================

fn millis() -> u64 {
    unsafe { sys::esp_timer_get_time() as u64 / 1000 }
}

// =============================================================================
// OFFLINE RTC BUFFER (ring buffer in RTC memory)
// =============================================================================

/// Sanitize RTC buffer indices (protection against brownout bit-flips).
fn sanitize_rtc_indices() {
    unsafe {
        if BUFFER_HEAD < 0 || BUFFER_HEAD >= MAX_OFFLINE_RECORDS as i32 {
            BUFFER_HEAD = 0;
        }
        if BUFFER_TAIL < 0 || BUFFER_TAIL >= MAX_OFFLINE_RECORDS as i32 {
            BUFFER_TAIL = 0;
        }
        if RTC_BUFFER_COUNT < 0 || RTC_BUFFER_COUNT > MAX_OFFLINE_RECORDS as i32 {
            RTC_BUFFER_COUNT = 0;
        }
    }
}

fn save_to_offline_buffer(data: &str) {
    sanitize_rtc_indices();
    unsafe {
        let head = BUFFER_HEAD as usize;
        let bytes = data.as_bytes();
        let len = bytes.len().min(MAX_LINE_LENGTH - 1);
        RTC_BUFFER[head][..len].copy_from_slice(&bytes[..len]);
        RTC_BUFFER[head][len] = 0;

        if RTC_BUFFER_COUNT < MAX_OFFLINE_RECORDS as i32 {
            RTC_BUFFER_COUNT += 1;
        } else {
            BUFFER_TAIL = (BUFFER_TAIL + 1) % MAX_OFFLINE_RECORDS as i32;
        }
        BUFFER_HEAD = (BUFFER_HEAD + 1) % MAX_OFFLINE_RECORDS as i32;
    }
}

fn flush_buffer_to_sd(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) {
    sanitize_rtc_indices();
    unsafe {
        if RTC_BUFFER_COUNT == 0 {
            return;
        }
    }

    brownout::enable();

    if !sdcard::mount() {
        return;
    }

    // Get filename from RTC
    let filename = if tca_select(i2c, CH_RTC, state) {
        match Ds3231::read_time(i2c) {
            Ok(dt) => dt.filename(),
            Err(_) => "/pomiar_unknown.csv".to_string(),
        }
    } else {
        "/pomiar_unknown.csv".to_string()
    };

    unsafe {
        let mut dummy_size = CURRENT_FILE_SIZE;
        for i in 0..RTC_BUFFER_COUNT {
            let idx = ((BUFFER_TAIL + i) % MAX_OFFLINE_RECORDS as i32) as usize;
            // Convert buffer to string
            let end = RTC_BUFFER[idx].iter().position(|&b| b == 0).unwrap_or(MAX_LINE_LENGTH);
            if let Ok(line) = std::str::from_utf8(&RTC_BUFFER[idx][..end]) {
                sdcard::write_line(&filename, line, &mut dummy_size);
            }
            feed_watchdog();
        }

        RTC_BUFFER_COUNT = 0;
        BUFFER_HEAD = 0;
        BUFFER_TAIL = 0;
        SD_FAIL_COUNT = 0;
        SD_RECOVERY_CYCLE = 0;
        SIZE_SYNCED = true;
        CURRENT_FILE_SIZE = dummy_size;
    }
}

// =============================================================================
// WIFI NTP SYNC
// =============================================================================

fn wifi_sync(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) {
    brownout::disable();

    info!("[{}] WiFi connecting to '{}'...", FW_VERSION, state.config_ssid);

    ensure_display(i2c, state);
    display_text(i2c, state, &["Laczenie..."]);

    if state.config_ssid.is_empty() {
        display_text(i2c, state, &["Brak SSID!", "Ustaw WiFi"]);
        FreeRtos::delay_ms(2000);
        brownout::enable();
        return;
    }

    unsafe {
        // Initialize WiFi in STA mode
        let mut wifi_cfg: sys::wifi_init_config_t = sys::wifi_init_config_default();
        if sys::esp_wifi_init(&wifi_cfg) != sys::ESP_OK as i32 {
            display_text(i2c, state, &["WiFi init ERR"]);
            FreeRtos::delay_ms(2000);
            brownout::enable();
            return;
        }

        sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA);

        // Build STA config with SSID + password
        let mut sta_cfg: sys::wifi_config_t = std::mem::zeroed();
        let ssid_bytes = state.config_ssid.as_bytes();
        let pass_bytes = state.config_pass.as_bytes();
        let ssid_len = ssid_bytes.len().min(31);
        let pass_len = pass_bytes.len().min(63);
        sta_cfg.sta.ssid[..ssid_len].copy_from_slice(
            &ssid_bytes[..ssid_len].iter().map(|&b| b as u8).collect::<Vec<u8>>()[..ssid_len],
        );
        sta_cfg.sta.password[..pass_len].copy_from_slice(
            &pass_bytes[..pass_len].iter().map(|&b| b as u8).collect::<Vec<u8>>()[..pass_len],
        );

        sys::esp_wifi_set_config(sys::wifi_interface_t_WIFI_IF_STA, &mut sta_cfg);
        sys::esp_wifi_start();
        sys::esp_wifi_connect();

        // Wait for connection (up to 10s)
        let start = millis();
        let mut connected = false;
        while millis() - start < 10_000 {
            feed_watchdog();
            FreeRtos::delay_ms(500);
            let mut ap_info: sys::wifi_ap_record_t = std::mem::zeroed();
            if sys::esp_wifi_sta_get_ap_info(&mut ap_info) == sys::ESP_OK as i32 {
                connected = true;
                break;
            }
        }

        if connected {
            info!("[{}] WiFi connected, starting SNTP...", FW_VERSION);

            // Configure SNTP
            let server = std::ffi::CString::new(NTP_SERVER).unwrap();
            sys::esp_sntp_setoperatingmode(sys::esp_sntp_operatingmode_t_ESP_SNTP_OPMODE_POLL);
            sys::esp_sntp_setservername(0, server.as_ptr() as *const i8);
            sys::esp_sntp_init();

            // Wait for NTP sync (up to 10s)
            let start = millis();
            let mut synced = false;
            while millis() - start < 10_000 {
                feed_watchdog();
                FreeRtos::delay_ms(500);
                let mut tv: libc::timeval = std::mem::zeroed();
                libc::gettimeofday(&mut tv, std::ptr::null_mut());
                // Time is considered synced if > 2024-01-01
                if tv.tv_sec > 1_704_067_200 {
                    synced = true;
                    break;
                }
            }

            if synced {
                // Read system time and adjust DS3231 RTC
                let mut now: libc::time_t = 0;
                libc::time(&mut now);
                // Apply timezone offset manually
                now += (GMT_OFFSET_SEC + DAYLIGHT_OFFSET_SEC) as libc::time_t;
                let mut tm: libc::tm = std::mem::zeroed();
                libc::gmtime_r(&now, &mut tm);

                let ntp_year = (tm.tm_year + 1900) as u16;

                // NTP validation — same as original: reject year outside 2024–2099
                if ntp_year >= 2024 && ntp_year <= 2099 {
                    let dt = DateTime {
                        year: ntp_year,
                        month: (tm.tm_mon + 1) as u8,
                        day: tm.tm_mday as u8,
                        hour: tm.tm_hour as u8,
                        minute: tm.tm_min as u8,
                        second: tm.tm_sec as u8,
                    };

                    if tca_select(i2c, CH_RTC, state) {
                        let _ = Ds3231::set_time(i2c, &dt);
                    }
                    display_text(i2c, state, &["Czas OK", &dt.timestamp_str()]);
                    info!("[{}] RTC synced: {}", FW_VERSION, dt.timestamp_str());
                } else {
                    warn!("[{}] NTP rejected: year={}", FW_VERSION, ntp_year);
                    display_text(i2c, state, &["NTP: BAD YEAR!"]);
                }
            } else {
                display_text(i2c, state, &["NTP timeout"]);
            }

            sys::esp_sntp_stop();
        } else {
            display_text(i2c, state, &["Brak WiFi"]);
        }

        FreeRtos::delay_ms(2000);

        // Disconnect and deinit WiFi
        sys::esp_wifi_disconnect();
        sys::esp_wifi_stop();
        sys::esp_wifi_deinit();
    }

    brownout::enable();
}

// =============================================================================
// INPUT KEYBOARD  (joystick-driven on-screen keyboard)
// =============================================================================

fn input_keyboard(
    i2c: &mut I2cDriver<'_>,
    state: &mut RuntimeState,
    title: &str,
    initial_value: &str,
) -> String {
    ensure_display(i2c, state);

    let chars: &[u8] = b"abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_+-=., ";
    let char_count = chars.len() as i32;
    let mut current_text = String::from(initial_value);
    let mut char_index: i32 = 0;
    let mut last_move = millis();
    let start_time_base = millis();
    let mut start_time = start_time_base;
    let max_chars_on_screen: usize = 16;

    loop {
        feed_watchdog();

        // UI Timeout
        if millis() - start_time > UI_TIMEOUT_MS {
            info!("[{}] inputKeyboard TIMEOUT", FW_VERSION);
            return current_text;
        }

        // Read joystick
        if !tca_select(i2c, CH_JOY, state) {
            FreeRtos::delay_ms(50);
            continue;
        }
        let x = QwiicJoystick::horizontal(i2c).unwrap_or(512);
        let y = QwiicJoystick::vertical(i2c).unwrap_or(512);
        let btn = QwiicJoystick::button(i2c).unwrap_or(1) == 0;

        if millis() - last_move > 150 {
            if y < 250 {
                char_index -= 1;
                if char_index < 0 { char_index = char_count - 1; }
                last_move = millis();
            }
            if y > 750 {
                char_index += 1;
                if char_index >= char_count { char_index = 0; }
                last_move = millis();
            }
            if x > 750 {
                if current_text.len() < 63 {
                    current_text.push(chars[char_index as usize] as char);
                    last_move = millis() + 250;
                    start_time = millis();
                }
            }
            if x < 250 {
                if !current_text.is_empty() {
                    current_text.pop();
                    last_move = millis() + 250;
                    start_time = millis();
                }
            }
        }

        if btn {
            FreeRtos::delay_ms(300);
            return current_text;
        }

        // Render to OLED
        if !tca_select(i2c, CH_OLED, state) {
            FreeRtos::delay_ms(50);
            continue;
        }

        let text_len = current_text.len();
        let view_offset = if text_len > max_chars_on_screen {
            text_len - max_chars_on_screen
        } else {
            0
        };

        let mut fb = [0u8; 1024];
        let mut tr = TextRenderer::new();
        tr.println(&mut fb, title);
        tr.set_cursor(0, 15);
        if view_offset > 0 {
            tr.print(&mut fb, "<");
        }
        let end = text_len.min(view_offset + max_chars_on_screen);
        let visible = &current_text[view_offset..end];
        tr.print(&mut fb, visible);

        // Draw cursor with inverted char
        let cursor_ch = chars[char_index as usize] as char;
        let mut ch_buf = [0u8; 4];
        let ch_str = cursor_ch.encode_utf8(&mut ch_buf);
        tr.print_inverted(&mut fb, ch_str);

        tr.set_cursor(0, 45);
        tr.print(&mut fb, "BTN: OK");
        let _ = Oled::flush(i2c, &fb);
    }
}

// =============================================================================
// SELECT NETWORK  (WiFi scan + joystick selection)
// =============================================================================

fn select_network(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) -> String {
    brownout::disable();
    ensure_display(i2c, state);
    display_text(i2c, state, &["Skanowanie..."]);

    // Scan for networks using raw esp-idf-sys
    let mut networks: Vec<String> = Vec::new();

    unsafe {
        let mut wifi_cfg: sys::wifi_init_config_t = sys::wifi_init_config_default();
        if sys::esp_wifi_init(&wifi_cfg) != sys::ESP_OK as i32 {
            display_text(i2c, state, &["WiFi init ERR"]);
            FreeRtos::delay_ms(2000);
            brownout::enable();
            return String::new();
        }
        sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA);
        sys::esp_wifi_start();
        feed_watchdog();

        let scan_cfg: sys::wifi_scan_config_t = std::mem::zeroed();
        sys::esp_wifi_scan_start(&scan_cfg, true);
        feed_watchdog();

        let mut ap_count: u16 = 0;
        sys::esp_wifi_scan_get_ap_num(&mut ap_count);

        if ap_count > 0 {
            let count = (ap_count as usize).min(20);
            let mut ap_records: Vec<sys::wifi_ap_record_t> = vec![std::mem::zeroed(); count];
            let mut found = count as u16;
            sys::esp_wifi_scan_get_ap_records(&mut found, ap_records.as_mut_ptr());

            for i in 0..found as usize {
                let ssid_bytes = &ap_records[i].ssid;
                let end = ssid_bytes.iter().position(|&b| b == 0).unwrap_or(ssid_bytes.len());
                if let Ok(s) = std::str::from_utf8(&ssid_bytes[..end]) {
                    if !s.is_empty() {
                        networks.push(s.to_string());
                    }
                }
            }
        }

        sys::esp_wifi_stop();
        sys::esp_wifi_deinit();
    }

    brownout::enable();

    if networks.is_empty() {
        display_text(i2c, state, &["Brak sieci!"]);
        FreeRtos::delay_ms(2000);
        return String::new();
    }

    let n = networks.len();
    let mut selected: i32 = 0;
    let mut last_move = millis();
    let select_start = millis();

    loop {
        feed_watchdog();

        if millis() - select_start > UI_TIMEOUT_MS {
            info!("[{}] selectNetwork TIMEOUT", FW_VERSION);
            return String::new();
        }

        if !tca_select(i2c, CH_JOY, state) {
            FreeRtos::delay_ms(50);
            continue;
        }
        let y = QwiicJoystick::vertical(i2c).unwrap_or(512);
        let btn = QwiicJoystick::button(i2c).unwrap_or(1) == 0;

        if millis() - last_move > 200 {
            if y < 250 {
                selected -= 1;
                if selected < 0 { selected = n as i32 - 1; }
                last_move = millis();
            }
            if y > 750 {
                selected += 1;
                if selected >= n as i32 { selected = 0; }
                last_move = millis();
            }
        }

        if btn {
            FreeRtos::delay_ms(300);
            return networks[selected as usize].clone();
        }

        if !tca_select(i2c, CH_OLED, state) {
            FreeRtos::delay_ms(50);
            continue;
        }

        let mut fb = [0u8; 1024];
        let mut tr = TextRenderer::new();
        tr.println(&mut fb, "Wybierz siec:");

        let start_idx = if selected > (n as i32 - 3) {
            (n as i32 - 3).max(0)
        } else {
            selected.max(0)
        } as usize;

        for i in 0..3 {
            let current = start_idx + i;
            if current >= n { break; }
            let prefix = if current == selected as usize { "> " } else { "  " };
            // Truncate SSID to 14 chars (like original)
            let ssid_display: String = networks[current].chars().take(14).collect();
            let line = format!("{}{}", prefix, ssid_display);
            tr.println(&mut fb, &line);
        }

        let _ = Oled::flush(i2c, &fb);
    }
}

// =============================================================================
// CALIBRATION FUNCTIONS
// =============================================================================

/// Calibrate NH3 zero-point (read 10 samples, average as baseline).
fn calibrate_nh3_zero(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) {
    ensure_display(i2c, state);
    display_text(i2c, state, &["Kalib. NH3..."]);

    if !tca_select(i2c, CH_ADS, state) || !state.ads_initialized {
        display_text(i2c, state, &["ADS ERR"]);
        FreeRtos::delay_ms(1500);
        return;
    }

    let mut sum: i64 = 0;
    for _ in 0..10 {
        match Ads1115::read_single_ended_ch0(i2c) {
            Ok(val) => sum += val as i64,
            Err(_) => {}
        }
        FreeRtos::delay_ms(100);
        feed_watchdog();
    }

    state.nh3_baseline = (sum / 10) as i32;
    let _ = preferences::put_i32("calib", "nh3_base", state.nh3_baseline);

    if tca_select(i2c, CH_OLED, state) {
        let msg = format!("OK! Base={}", state.nh3_baseline);
        display_text(i2c, state, &["Kalib. NH3...", &msg]);
    }
    FreeRtos::delay_ms(1500);
}

/// Calibrate CO2 sensor (400 ppm zero-point).
fn calibrate_co2(
    i2c: &mut I2cDriver<'_>,
    co2_uart: &mut UartDriver<'_>,
    mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    state: &mut RuntimeState,
) {
    ensure_display(i2c, state);
    display_text(i2c, state, &["KALIBRACJA CO2", "400 ppm", "[BTN] Start"]);

    // Wait for button press
    let timeout = millis();
    loop {
        feed_watchdog();
        if millis() - timeout > 30_000 { return; }
        if tca_select(i2c, CH_JOY, state) {
            if QwiicJoystick::button(i2c).unwrap_or(1) == 0 { break; }
        }
        FreeRtos::delay_ms(10);
    }
    // Wait for button release
    let timeout = millis();
    loop {
        feed_watchdog();
        if millis() - timeout > 30_000 { return; }
        if tca_select(i2c, CH_JOY, state) {
            if QwiicJoystick::button(i2c).unwrap_or(1) != 0 { break; }
        }
        FreeRtos::delay_ms(10);
    }

    power_mhz19_on(mosfet, co2_uart);

    display_text(i2c, state, &["Wait..."]);
    Mhz19::calibrate(co2_uart);
    feed_watchdog();
    FreeRtos::delay_ms(2000);
    feed_watchdog();
    display_text(i2c, state, &["GOTOWE!"]);
    FreeRtos::delay_ms(2000);
}

/// Set NH3 sensitivity factor via keyboard.
fn set_nh3_factor(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) {
    let current_val = format!("{:.5}", state.nh3_factor);
    let input = input_keyboard(i2c, state, "Faktor (0.0-0.1)", &current_val);

    if !input.is_empty() {
        if let Ok(new_factor) = input.parse::<f32>() {
            if new_factor > 0.00001 && new_factor < 0.1 {
                state.nh3_factor = new_factor;
                let _ = preferences::put_f32("calib", "nh3_factor", state.nh3_factor);

                ensure_display(i2c, state);
                let msg = format!("Nowy F: {:.5}", state.nh3_factor);
                display_text(i2c, state, &["ZAPISANO!", &msg]);
                FreeRtos::delay_ms(2000);
                return;
            }
        }
        ensure_display(i2c, state);
        display_text(i2c, state, &["BLAD ZAKRESU!"]);
        FreeRtos::delay_ms(2000);
    }
}

// =============================================================================
// SERVICE MODE  (multi-screen joystick menu)
// =============================================================================

fn run_service_mode(
    i2c: &mut I2cDriver<'_>,
    co2_uart: &mut UartDriver<'_>,
    mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    state: &mut RuntimeState,
) {
    ensure_display(i2c, state);

    let mut screen: i32 = 0;
    let mut last_act = millis();
    let mut last_move = millis();

    loop {
        feed_watchdog();

        // Read joystick
        if tca_select(i2c, CH_JOY, state) {
            let y = QwiicJoystick::vertical(i2c).unwrap_or(512);
            let btn = QwiicJoystick::button(i2c).unwrap_or(1) == 0;

            if millis() - last_move > 250 {
                if y < 250 { screen -= 1; last_move = millis(); last_act = millis(); }
                if y > 750 { screen += 1; last_move = millis(); last_act = millis(); }
                if screen < 0 { screen = 2; }
                if screen > 2 { screen = 0; }
            }

            // Button handling for screen 2 (MENU)
            if btn && millis() - last_act > 500 {
                last_act = millis();
                if screen == 2 {
                    run_service_submenu(i2c, co2_uart, mosfet, state);
                    last_act = millis();
                }
            }
        }

        // Render current screen
        if !tca_select(i2c, CH_OLED, state) {
            FreeRtos::delay_ms(50);
            continue;
        }

        let mut fb = [0u8; 1024];
        let mut tr = TextRenderer::new();

        match screen {
            0 => {
                // STATUS screen
                let header = format!("[ STATUS {} AK ]", FW_VERSION);
                tr.println(&mut fb, &header);

                unsafe {
                    let sd_line = if SD_FAIL_COUNT > 5 {
                        format!("SD: RECOVERY ({})", SD_RECOVERY_CYCLE)
                    } else if SD_FAIL_COUNT > 0 {
                        format!("SD: ERR {}", SD_FAIL_COUNT)
                    } else {
                        "SD: OK".to_string()
                    };
                    tr.println(&mut fb, &sd_line);

                    let buf_line = format!("Buf: {} FSz: {}KB", RTC_BUFFER_COUNT, CURRENT_FILE_SIZE / 1024);
                    tr.println(&mut fb, &buf_line);

                    let mut wake_line = format!("Wake: {}", WAKE_COUNT);
                    if state.mux_err_count > 0 {
                        write!(wake_line, " MX:{}", state.mux_err_count).ok();
                    }
                    tr.println(&mut fb, &wake_line);

                    if SVC_MODE_STREAK > SVC_MODE_STREAK_MAX {
                        tr.println(&mut fb, "JOY: STUCK!");
                    }
                }
            }
            1 => {
                // LIVE screen
                tr.println(&mut fb, "[ LIVE ]");

                if tca_select(i2c, CH_ADS, state) && state.ads_initialized {
                    match Ads1115::read_single_ended_ch0(i2c) {
                        Ok(r) => {
                            let cal = r as i32 - state.nh3_baseline;
                            let n_ppm = if cal < 0 { 0.0 } else { cal as f32 * state.nh3_factor };
                            if tca_select(i2c, CH_OLED, state) {
                                let line = format!("NH3: {:.3} ppm", n_ppm);
                                tr.println(&mut fb, &line);
                            }
                        }
                        Err(_) => {
                            if tca_select(i2c, CH_OLED, state) {
                                tr.println(&mut fb, "NH3: READ ERR");
                            }
                        }
                    }
                } else {
                    tr.println(&mut fb, "NH3: MUX ERR");
                }

                let c_ppm = get_safe_co2(co2_uart);
                if tca_select(i2c, CH_OLED, state) {
                    if c_ppm > 0 {
                        let line = format!("CO2: {} ppm", c_ppm);
                        tr.println(&mut fb, &line);
                    } else {
                        tr.println(&mut fb, "CO2: OFF/ERR");
                    }
                }
            }
            _ => {
                // MENU entry screen
                tr.println(&mut fb, "[ MENU ]");
                tr.println(&mut fb, "[KLIK] Wejdz");
            }
        }

        let _ = Oled::flush(i2c, &fb);

        // 30s inactivity timeout
        if millis() - last_act > 30_000 {
            break;
        }
        FreeRtos::delay_ms(50);
    }
}

/// Service mode submenu with 6 options (same as original).
fn run_service_submenu(
    i2c: &mut I2cDriver<'_>,
    co2_uart: &mut UartDriver<'_>,
    mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    state: &mut RuntimeState,
) {
    let menu_items = ["Sync Czas", "WiFi Setup", "Kalib. NH3 (Zero)", "Kalib. CO2", "Ustaw Faktor NH3", "Wyjscie"];
    let menu_count = menu_items.len() as i32;
    let mut m_idx: i32 = 0;
    let mut last_move = millis();
    let mut last_act = millis();

    loop {
        feed_watchdog();

        if !tca_select(i2c, CH_JOY, state) {
            FreeRtos::delay_ms(50);
            continue;
        }
        let sy = QwiicJoystick::vertical(i2c).unwrap_or(512);
        let btn = QwiicJoystick::button(i2c).unwrap_or(1) == 0;

        if millis() - last_move > 200 {
            if sy < 250 {
                m_idx -= 1;
                if m_idx < 0 { m_idx = menu_count - 1; }
                last_move = millis();
            }
            if sy > 750 {
                m_idx += 1;
                if m_idx >= menu_count { m_idx = 0; }
                last_move = millis();
            }
        }

        // Render menu
        if tca_select(i2c, CH_OLED, state) {
            let mut fb = [0u8; 1024];
            let mut tr = TextRenderer::new();
            tr.println(&mut fb, "[ MENU GLOWNE ]");

            let start_list = if m_idx > 3 { (m_idx - 3) as usize } else { 0 };
            for i in 0..4 {
                let item_index = start_list + i;
                if item_index < menu_items.len() {
                    tr.set_cursor(5, 14 + (i * 12));
                    let prefix = if m_idx == item_index as i32 { "> " } else { "  " };
                    let line = format!("{}{}", prefix, menu_items[item_index]);
                    tr.print(&mut fb, &line);
                }
            }

            // Scrollbar
            let scroll_y = 14 + ((m_idx as usize * 36) / (menu_count as usize - 1).max(1));
            TextRenderer::fill_rect(&mut fb, 124, scroll_y, 3, 3, true);

            let _ = Oled::flush(i2c, &fb);
        }

        // Handle button press
        if btn {
            FreeRtos::delay_ms(200);
            match m_idx {
                0 => wifi_sync(i2c, state),
                1 => {
                    let ssid = select_network(i2c, state);
                    if !ssid.is_empty() {
                        let pass = input_keyboard(i2c, state, "Haslo WiFi", "");
                        let _ = preferences::put_string("wifi", "ssid", &ssid);
                        let _ = preferences::put_string("wifi", "pass", &pass);
                        state.config_ssid = ssid;
                        state.config_pass = pass;
                    }
                }
                2 => calibrate_nh3_zero(i2c, state),
                3 => calibrate_co2(i2c, co2_uart, mosfet, state),
                4 => set_nh3_factor(i2c, state),
                5 => return, // Wyjscie
                _ => {}
            }
            last_act = millis();
            // Clear display after menu action
            if tca_select(i2c, CH_OLED, state) {
                let fb = [0u8; 1024];
                let _ = Oled::flush(i2c, &fb);
            }
        }

        // 30s inactivity timeout
        if millis() - last_act > 30_000 {
            return;
        }
    }
}

fn get_safe_co2(co2_uart: &mut UartDriver<'_>) -> i32 {
    feed_watchdog();
    if let Some(ppm) = Mhz19::read_co2(co2_uart) {
        return ppm;
    }

    // Rescue attempt
    warn!("[{}] CO2 rescue attempt...", FW_VERSION);
    FreeRtos::delay_ms(500);
    feed_watchdog();

    if let Some(ppm) = Mhz19::read_co2(co2_uart) {
        return ppm;
    }

    -1
}

// =============================================================================
// POWER GATING (MH-Z19 via P-MOSFET)
// =============================================================================

fn power_mhz19_on(
    mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    co2_uart: &mut UartDriver<'_>,
) {
    info!("[{}] MH-Z19 POWER ON — start preheat", FW_VERSION);
    mosfet.set_low().ok();

    FreeRtos::delay_ms(500);
    Mhz19::auto_calibration_off(co2_uart);

    let borrowed_ms = (FAN_DURATION_SEC + SETTLE_DURATION_SEC) * 1000;
    let extra_wait_ms = (MHZ19_PREHEAT_SEC * 1000).saturating_sub(borrowed_ms);

    if extra_wait_ms > 0 {
        info!("[{}] MH-Z19 extra preheat: {}s", FW_VERSION, extra_wait_ms / 1000);
        let start = millis();
        while millis() - start < extra_wait_ms {
            FreeRtos::delay_ms(1000);
            feed_watchdog();
        }
    }
}

fn power_mhz19_off(mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>) {
    info!("[{}] MH-Z19 POWER OFF", FW_VERSION);
    mosfet.set_high().ok();
}

// =============================================================================
// DISPLAY HELPERS
// =============================================================================

fn ensure_display(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) {
    if !state.display_ready {
        if tca_select(i2c, CH_OLED, state) {
            if Oled::init(i2c).is_ok() {
                state.display_ready = true;
            }
        }
    }
}

fn display_text(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState, lines: &[&str]) {
    if !state.display_ready {
        return;
    }
    if !tca_select(i2c, CH_OLED, state) {
        return;
    }
    let mut fb = [0u8; 1024];
    let mut tr = TextRenderer::new();
    for line in lines {
        tr.println(&mut fb, line);
    }
    let _ = Oled::flush(i2c, &fb);
}

// =============================================================================
// GO TO SLEEP
// =============================================================================

fn go_to_sleep(
    i2c: &mut I2cDriver<'_>,
    mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    fan: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    pms_uart: &mut UartDriver<'_>,
    state: &mut RuntimeState,
    sleep_us: u64,
) -> ! {
    unsafe {
        info!(
            ">>> SLEEP {} AK | wake#{} | MUX_ERR:{} <<<",
            FW_VERSION, WAKE_COUNT, state.mux_err_count
        );
    }

    power_mhz19_off(mosfet);
    fan.set_low().ok();

    Pms5003::sleep(pms_uart);

    // Turn off OLED (MUX-safe)
    if state.display_ready {
        if tca_select(i2c, CH_OLED, state) {
            let mut fb = [0u8; 1024];
            Oled::clear(&mut fb);
            let _ = Oled::flush(i2c, &fb);
            let _ = Oled::display_off(i2c);
        } else {
            warn!("[{}] WARN: MUX FAIL — OLED charge pump may stay ON!", FW_VERSION);
        }
    }

    sdcard::unmount();

    deep_sleep_us(sleep_us)
}

// =============================================================================
// CALCULATE AND SET WAKEUP
// =============================================================================

fn calculate_sleep_us(i2c: &mut I2cDriver<'_>, state: &mut RuntimeState) -> u64 {
    if !tca_select(i2c, CH_RTC, state) {
        return 15 * 60 * 1_000_000; // fallback 15 min
    }
    let now = match Ds3231::read_time(i2c) {
        Ok(dt) => dt,
        Err(_) => return 15 * 60 * 1_000_000,
    };

    let cur = now.minute as i64 * 60 + now.second as i64;
    let tgt = if cur < 900 {
        900
    } else if cur < 1800 {
        1800
    } else if cur < 2700 {
        2700
    } else {
        3600
    };

    let mut sleep_sec = tgt - cur - WAKEUP_OFFSET_SEC as i64;
    if sleep_sec < 10 {
        sleep_sec += 900;
    }
    (sleep_sec as u64) * 1_000_000
}

// =============================================================================
// MAIN MEASUREMENT SEQUENCE
// =============================================================================

fn run_measurement_sequence(
    i2c: &mut I2cDriver<'_>,
    co2_uart: &mut UartDriver<'_>,
    pms_uart: &mut UartDriver<'_>,
    mosfet: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    fan: &mut PinDriver<'_, impl OutputPin, gpio::Output>,
    state: &mut RuntimeState,
) {
    feed_watchdog();

    // 1. Flush any offline buffer records to SD
    flush_buffer_to_sd(i2c, state);

    // 2. Power on CO2 sensor
    power_mhz19_on(mosfet, co2_uart);

    // 3. Fan cycle
    fan.set_high().ok();
    let start = millis();
    while millis() - start < FAN_DURATION_SEC * 1000 {
        FreeRtos::delay_ms(100);
        feed_watchdog();
    }
    fan.set_low().ok();

    // 4. Settle (minus PMS warmup overlap)
    let settle_wait = (SETTLE_DURATION_SEC - PMS_WARMUP_SEC) * 1000;
    let start = millis();
    while millis() - start < settle_wait {
        FreeRtos::delay_ms(100);
        feed_watchdog();
    }

    // 5. Wake PMS sensor
    Pms5003::wake(pms_uart);
    let start = millis();
    while millis() - start < PMS_WARMUP_SEC * 1000 {
        FreeRtos::delay_ms(100);
        feed_watchdog();
    }

    // 6. Read all sensors
    let mut temp: f32 = f32::NAN;
    let mut hum: f32 = f32::NAN;
    let mut nh3_raw: i16 = 0;
    let mut nh3_ppm: f32 = 0.0;
    let mut pm = PmsData::default();
    let mut ts = "RTC_ERR".to_string();

    // CO2
    let co2_ppm = get_safe_co2(co2_uart);

    // Temperature & Humidity (AHT20)
    if tca_select(i2c, CH_AHT, state) {
        match Aht20::read(i2c) {
            Ok((t, h)) => {
                temp = t;
                hum = h;
            }
            Err(e) => warn!("[{}] AHT read error: {:?}", FW_VERSION, e),
        }
    } else {
        warn!("[{}] MUX FAIL: AHT — temp/hum = NAN", FW_VERSION);
    }

    // NH3 (ADS1115)
    if tca_select(i2c, CH_ADS, state) {
        if state.ads_initialized {
            match Ads1115::read_single_ended_ch0(i2c) {
                Ok(raw) => {
                    nh3_raw = raw;
                    if raw < 32700 && raw > -1000 {
                        let cal = raw as i32 - state.nh3_baseline;
                        nh3_ppm = if cal < 0 { 0.0 } else { cal as f32 * state.nh3_factor };
                    } else {
                        nh3_ppm = -1.0;
                    }
                }
                Err(e) => {
                    warn!("[{}] ADS read error: {:?}", FW_VERSION, e);
                    nh3_ppm = -1.0;
                }
            }
        }
    } else {
        warn!("[{}] MUX FAIL: ADS — NH3 = -1", FW_VERSION);
        nh3_ppm = -1.0;
    }

    // Particulate Matter (PMS5003)
    if let Some(data) = Pms5003::read(pms_uart, 4000) {
        pm = data;
    }
    Pms5003::sleep(pms_uart);

    // RTC timestamp & SD write
    if tca_select(i2c, CH_RTC, state) {
        let (timestamp, filename) = match Ds3231::read_time(i2c) {
            Ok(dt) => (dt.timestamp_str(), dt.filename()),
            Err(_) => ("RTC_ERR".to_string(), "/pomiar_unknown.csv".to_string()),
        };
        ts = timestamp;

        let line = format!(
            "{},{:.2},{:.2},{},{:.3},{},{},{},{}",
            ts, temp, hum, nh3_raw, nh3_ppm, co2_ppm, pm.pm1_0, pm.pm2_5, pm.pm10
        );

        unsafe {
            brownout::enable();
            if sdcard::mount() {
                let mut fsize = CURRENT_FILE_SIZE;
                if sdcard::write_line(&filename, &line, &mut fsize) {
                    SD_FAIL_COUNT = 0;
                    CURRENT_FILE_SIZE = fsize;
                } else {
                    SD_FAIL_COUNT += 1;
                    save_to_offline_buffer(&line);
                }
            } else {
                SD_FAIL_COUNT += 1;
                save_to_offline_buffer(&line);
            }
        }
    } else {
        warn!("[{}] MUX FAIL: RTC — using fallback timestamp", FW_VERSION);
        let line = format!(
            "MUX_RTC_ERR,{:.2},{:.2},{},{:.3},{},{},{},{}",
            temp, hum, nh3_raw, nh3_ppm, co2_ppm, pm.pm1_0, pm.pm2_5, pm.pm10
        );
        save_to_offline_buffer(&line);
    }

    // Power off CO2
    power_mhz19_off(mosfet);

    // 7. Display results
    ensure_display(i2c, state);
    let mux_alive = tca_select(i2c, CH_OLED, state);
    let w_start = millis();

    if !mux_alive {
        info!("[{}] MUX dead in display loop — blind wait", FW_VERSION);
        while millis() - w_start < DISPLAY_RESULT_MS {
            FreeRtos::delay_ms(100);
            feed_watchdog();
        }
    } else {
        while millis() - w_start < DISPLAY_RESULT_MS {
            feed_watchdog();

            // Check joystick for service mode entry
            if tca_select(i2c, CH_JOY, state) {
                unsafe {
                    if let Ok(0) = QwiicJoystick::button(i2c) {
                        if SVC_MODE_STREAK <= SVC_MODE_STREAK_MAX {
                            FreeRtos::delay_ms(1000);
                            if let Ok(0) = QwiicJoystick::button(i2c) {
                                SVC_MODE_STREAK += 1;
                                run_service_mode(i2c, co2_uart, mosfet, state);
                                return;
                            }
                        }
                    }
                }
            }

            // Update display
            if tca_select(i2c, CH_OLED, state) {
                let sd_status = unsafe {
                    if SD_FAIL_COUNT > 5 {
                        "! SD RECOVERY !"
                    } else if SD_FAIL_COUNT > 0 {
                        "! SD ERROR !"
                    } else {
                        "SD OK"
                    }
                };
                let co2_line = format!("ZAPISANO. CO2: {}", co2_ppm);
                display_text(i2c, state, &[&co2_line, sd_status, "", "MENU? Przytrzymaj BTN"]);
            }
            FreeRtos::delay_ms(100);
        }
    }
}

// =============================================================================
// MAIN ENTRY POINT
// =============================================================================

fn main() -> Result<()> {
    // Initialize ESP-IDF logging
    esp_idf_svc::log::EspLogger::initialize_default();

    init_watchdog();

    unsafe {
        WAKE_COUNT += 1;
        info!("[{}] Wake #{}", FW_VERSION, WAKE_COUNT);
    }

    // ---- Peripherals ----
    let peripherals = Peripherals::take()?;

    // Fan pin
    let mut fan = PinDriver::output(peripherals.pins.gpio4)?;
    fan.set_low()?;

    // MOSFET pin (MH-Z19 power gate)
    let mut mosfet = PinDriver::output(peripherals.pins.gpio13)?;
    mosfet.set_high()?; // MH-Z19 OFF

    // I2C bus (SDA=21, SCL=22)
    let i2c_config = I2cConfig::new().baudrate(100.kHz().into());
    let mut i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio21,
        peripherals.pins.gpio22,
        &i2c_config,
    )?;

    // CO2 UART (32=RX, 33=TX, 9600 baud)
    let co2_uart_config = UartConfig::new().baudrate(Hertz(9600));
    let mut co2_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio33, // TX
        peripherals.pins.gpio32, // RX
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &co2_uart_config,
    )?;

    // PMS UART (16=RX, 17=TX, 9600 baud)
    let pms_uart_config = UartConfig::new().baudrate(Hertz(9600));
    let mut pms_uart = UartDriver::new(
        peripherals.uart2,
        peripherals.pins.gpio17, // TX
        peripherals.pins.gpio16, // RX
        Option::<gpio::Gpio2>::None,
        Option::<gpio::Gpio3>::None,
        &pms_uart_config,
    )?;

    // ---- Runtime State ----
    let mut state = RuntimeState::default();

    // Init joystick
    if tca_select(&mut i2c, CH_JOY, &mut state) {
        state.joy_initialized = QwiicJoystick::begin(&mut i2c);
    }

    // Init ADS1115
    if tca_select(&mut i2c, CH_ADS, &mut state) {
        // Try reading config register to verify presence
        let mut buf = [0u8; 2];
        if i2c.write_read(ADS_ADDR, &[0x01], &mut buf, 100).is_ok() {
            state.ads_initialized = true;
        }
    }

    // Load calibration from NVS
    state.nh3_baseline = preferences::get_i32("calib", "nh3_base", 0);
    state.nh3_factor = preferences::get_f32("calib", "nh3_factor", DEFAULT_NH3_FACTOR);

    // Load WiFi credentials
    state.config_ssid = preferences::get_string("wifi", "ssid", "");
    state.config_pass = preferences::get_string("wifi", "pass", "");

    brownout::save_once();

    // ---- Stuck joystick protection ----
    if state.joy_initialized {
        if tca_select(&mut i2c, CH_JOY, &mut state) {
            unsafe {
                match QwiicJoystick::button(&mut i2c) {
                    Ok(0) => {
                        // Button pressed on wakeup
                        SVC_MODE_STREAK += 1;
                        if SVC_MODE_STREAK <= SVC_MODE_STREAK_MAX {
                            info!(
                                "[{}] ServiceMode entry (streak: {})",
                                FW_VERSION, SVC_MODE_STREAK
                            );
                            run_service_mode(
                                &mut i2c,
                                &mut co2_uart,
                                &mut mosfet,
                                &mut state,
                            );
                            // After service mode → calculate wakeup → sleep
                            let sleep_us = calculate_sleep_us(&mut i2c, &mut state);
                            go_to_sleep(
                                &mut i2c, &mut mosfet, &mut fan,
                                &mut pms_uart, &mut state, sleep_us,
                            );
                        } else {
                            info!(
                                "[{}] BTN STUCK — ignoring (streak: {})",
                                FW_VERSION, SVC_MODE_STREAK
                            );
                        }
                    }
                    _ => {
                        // Button free — reset streak
                        if SVC_MODE_STREAK > 0 {
                            info!(
                                "[{}] BTN released, streak reset (was: {})",
                                FW_VERSION, SVC_MODE_STREAK
                            );
                        }
                        SVC_MODE_STREAK = 0;
                    }
                }
            }
        }
    }

    // ---- Run measurement ----
    run_measurement_sequence(
        &mut i2c,
        &mut co2_uart,
        &mut pms_uart,
        &mut mosfet,
        &mut fan,
        &mut state,
    );

    // ---- Calculate next wakeup and sleep ----
    let sleep_us = calculate_sleep_us(&mut i2c, &mut state);
    go_to_sleep(&mut i2c, &mut mosfet, &mut fan, &mut pms_uart, &mut state, sleep_us);
}
