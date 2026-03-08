# Smart Stables Monitor == Rust Port

## Opis

Port firmware `stables_1_0_0` z Arduino C++ (.ino) na Rust, używając ekosystemu `esp-idf-hal` / `esp-idf-svc`.

**Autor oryginału**: Maciej Kasperek (vonKrappitz)  
**Wersja**: 1.0.0

## Architektura

| Moduł C++ (Arduino)         | Odpowiednik Rust                         |
|------------------------------|------------------------------------------|
| `Wire.h` / I2C              | `esp-idf-hal::i2c::I2cDriver`           |
| `Adafruit_SSD1306`          | Własny driver `Oled` + `TextRenderer`    |
| `Adafruit_AHTX0`            | Własny driver `Aht20`                    |
| `RTClib` (DS3231)            | Własny driver `Ds3231`                   |
| `Adafruit_ADS1X15`          | Własny driver `Ads1115`                  |
| `SparkFun_Qwiic_Joystick`   | Własny driver `QwiicJoystick`            |
| `MHZ19` (CO2, UART)         | Własny driver `Mhz19`                   |
| `PMS.h` (PM sensor, UART)   | Własny driver `Pms5003`                 |
| `SD.h` / FAT32              | `esp_vfs_fat_sdspi_mount` (raw FFI)      |
| `Preferences`               | `esp-idf-svc::nvs::EspNvs`              |
| `RTC_DATA_ATTR`             | `#[link_section = ".rtc.data"]`          |
| `esp_task_wdt`              | bezpośrednie FFI do `esp-idf-sys`        |
| `esp_deep_sleep_start`      | bezpośrednie FFI do `esp-idf-sys`        |
| Brownout reg manipulation   | surowe odczyty/zapisy rejestrów          |

## Co zostało w pełni przeportowane

- ✅ Cała logika pomiarowa (`runMeasurementSequence`)
- ✅ Wszystkie drivery sensorów (AHT20, DS3231, ADS1115, MH-Z19, PMS5003)
- ✅ I2C MUX (TCA9548A) z obsługą błędów
- ✅ SSD1306 OLED == init, clear, flush, pełny font 5×7 (95 glifów ASCII)
- ✅ TextRenderer z `fill_rect`, `print_inverted`, `progress_bar`
- ✅ RTC ring buffer z hardeningiem (sanityzacja indeksów)
- ✅ SD card mount/write via ESP-IDF VFS
- ✅ NVS preferences (kalibracja NH3, WiFi credentials)
- ✅ Deep sleep z wakeup timer
- ✅ Watchdog (45s timeout)
- ✅ Brownout save/disable/enable
- ✅ MOSFET power gating (MH-Z19)
- ✅ Stuck joystick protection (svcModeStreak)
- ✅ CO2 rescue protocol (getSafeCO2)
- ✅ File size cache, SD recovery mechanism
- ✅ WiFi NTP Sync (pełna implementacja via esp-idf-sys FFI)
- ✅ Service Mode == 3 ekrany (STATUS, LIVE, MENU) + joystick navigation
- ✅ Service Submenu == 6 opcji (Sync Czas, WiFi Setup, Kalib NH3, Kalib CO2, Faktor NH3, Wyjście)
- ✅ Input Keyboard == joystick-driven ekranowa klawiatura z scrollowaniem
- ✅ Select Network == skanowanie WiFi + lista SSID
- ✅ Calibrate NH3 Zero == 10-sample averaging + zapis do NVS
- ✅ Calibrate CO2 == 400ppm zero-point via MH-Z19 UART command
- ✅ Set NH3 Factor == keyboard input z walidacją zakresu

## Uwagi implementacyjne

- ⚠️ **SD Card SPI config** == `sdcard::mount()` używa surowego FFI. Piny SPI (MOSI/MISO/CLK) muszą być dopasowane do hardware'u. Domyślne ESP32 SPI2 to: MOSI=23, MISO=19, CLK=18.
- ⚠️ **WiFi** == implementacja używa surowego `esp-idf-sys` FFI zamiast `esp-idf-svc::wifi` (wymagałoby przekazywania `peripherals.modem`). Funkcjonalnie identyczne z oryginałem.
- ⚠️ **NVS** == moduł `preferences` wymaga dostępności partycji NVS w flash. Upewnij się, że partition table ma wpis `nvs`.

## Jak zbudować

```bash
# Wymagane: ESP-IDF v5.x + Rust toolchain dla xtensa
# https://github.com/esp-rs/esp-idf-template

cargo build --release --target xtensa-esp32-espidf
```

## Struktura plików

```
smart_stables/
├── Cargo.toml          # Zależności i konfiguracja
├── src/
│   └── main.rs         # Cały firmware (single-file port)
└── README.md           # Ten plik
```

## Różnice vs oryginał C++

1. **Bezpieczeństwo typów** == Rust wymusza obsługę błędów (`Result<T>`) zamiast cichego ignorowania.
2. **Borrow checker** == I2C driver jest przekazywany explicite, nie przez globalne obiekty.
3. **Brak `loop()`** == w Arduino `setup()` robiło całą pracę, a `loop()` był pusty. W Ruście `main()` kończy się deep sleep → nigdy nie wraca.
4. **Modularność** == drivery sensorów to osobne struktury z metodami.
