/*
 * PROJECT / PROJEKT: Smart Stables Monitor
 * AUTHORS / AUTORZY: Maciej Kasperek (von Krappitz)
 * VERSION / WERSJA: 1.0.0 Stable (Hardening II - formerly internal dev v3.9)
 * DATE / DATA: 2026-02-22
 *
 * =====================================================================================
 * CHANGELOG 1.0.0 (vs internal dev v3.8) / OPIS ZMIAN 1.0.0 (względem v3.8):
 * =====================================================================================
 *
 * 1. [HARDENING] RTC Memory Protection / Ochrona pamięci RTC
 * EN: Sanitize bufferHead/Tail/Count to prevent out-of-bounds writes due to brownout bit-flips.
 * PL: Sanityzacja indeksów bufora. Brownout/bit-flip mógł skorumpować pamięć i wywołać crash.
 *
 * 2. [BUG-FIX] Stuck joystick protection / Pełne naprawienie logiki streak
 * EN: svcModeStreak is reset ONLY when the button is physically released, preventing oscillating lockups.
 * PL: svcModeStreak zerowany TYLKO gdy przycisk jest fizycznie wolny. Zapobiega to oscylacji menu.
 *
 * 3. [HARDENING] MUX-safe OLED Power Off / Bezpieczne wyłączanie OLED w sleep
 * EN: Checks if MUX is alive before sending I2C displayOff commands. Saves ~20mA parasitic drain.
 * PL: Sprawdza MUX przed wyłączeniem OLED. Chroni przed utratą prądu (do 20mA) przez pompę ładunkową.
 *
 * 4. [HARDENING] MUX-safe display loop / MUX-safe pętla wyświetlania wyniku
 * EN: Single MUX health check before loop prevents massive I2C timeouts (2.5s wasted) if MUX dies.
 * PL: Jeden test muxa przed pętlą. Zapobiega utracie 2.5s na I2C timeouty w przypadku awarii.
 *
 * =====================================================================================
 * POWER BALANCE / BILANS ENERGETYCZNY:
 * =====================================================================================
 * Active / Aktywny: 4.4h/d @ 145mA  = 642 mAh/d
 * Sleep / Uśpienie: 19.6h/d @ 12mA   = 235 mAh/d
 * TOTAL / RAZEM:                     = 877 mAh/d
 * Estimated Life / Szacunkowy czas: ~34 days/dni (30000 mAh battery)
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <time.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include "RTClib.h"
#include <PMS.h>
#include <MHZ19.h>              
#include <Adafruit_ADS1X15.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h> 
#include <esp_task_wdt.h>       

// LOW-LEVEL REGISTERS / NISKOPOZIOMOWE REJESTRY
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_arduino_version.h> 

// --- CONSTANTS & CONFIGURATION / KONFIGURACJA STAŁYCH ---

// Firmware Version String / Centralny string wersji
#define FW_VERSION "1.0.0"

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// Timings (Tuned for sensor chemical stability) / Timingi (Dostrojone pod stabilność chemiczną)
#define WAKEUP_OFFSET_SEC 200       
#define FAN_DURATION_SEC 15         
#define SETTLE_DURATION_SEC 105     
#define PMS_WARMUP_SEC 30           

// MOSFET Power Gate for MH-Z19 / Bramka zasilania
#define PIN_MOSFET 13               // GPIO13: P-MOSFET gate (IRLML6401)
                                     // LOW = MH-Z19 ON, HIGH = MH-Z19 OFF
#define MHZ19_PREHEAT_SEC 180       // NDIR warmup time / Rozgrzewka NDIR

// OLED Display Time / Czas wyświetlania wyniku na OLED
#define DISPLAY_RESULT_MS 5000      // 5 sec / 5 sekund

// Memory & File Limits / Limity pamięci i plików
#define MAX_OFFLINE_RECORDS 10      // RTC RAM buffer size / Maksymalny rozmiar bufora RTC
#define MAX_LINE_LENGTH 120         
#define MAX_FILE_SIZE_BYTES 52428800 // 50MB (FAT32 limit)

// SD Recovery Interval / Próba odzyskania karty SD
#define SD_RECOVERY_INTERVAL 10     // Every 10 wakeups / Co 10 wybudzeń

// Stuck joystick protection / Zabezpieczenie przed zablokowanym joystickiem
#define SVC_MODE_STREAK_MAX 3       // Ignore button after 3 enters / Ignoruj po 3 wejściach

// UI Timeouts / Limity czasu interfejsu
#define UI_TIMEOUT_MS 60000         // 60s timeout

#define DEFAULT_NH3_FACTOR 0.0025   

// --- PINOUT ---
#define PIN_FAN 4       
#define PIN_CS_SD 5   
#define PIN_PMS_RX 16   
#define PIN_PMS_TX 17   
#define PIN_CO2_RX 32   
#define PIN_CO2_TX 33   

const char* CSV_HEADER = "Timestamp,Temp_C,Humidity_%,NH3_Raw,NH3_PPM,CO2_ppm,PM1,PM2.5,PM10";

// --- I2C ADDRESSES / ADRESY I2C ---
#define MUX_ADDR 0x70
#define CH_OLED 0
#define CH_AHT 1
#define CH_RTC 2
#define CH_ADS 3     
#define CH_JOY 4     

// --- OBJECTS / OBIEKTY ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_AHTX0 aht;
RTC_DS3231 rtc;
PMS pms(Serial2);     
PMS::DATA pmsData;
MHZ19 myMHZ19;
HardwareSerial co2Serial(1); 
Preferences pref;       
Preferences calibPref;  
Adafruit_ADS1115 ads; 
JOYSTICK joystick;

// =================================================================================
// RTC VARIABLES (BATTERY-BACKED NON-VOLATILE) / ZMIENNE RTC (PAMIĘĆ TRWAŁA)
// =================================================================================

RTC_DATA_ATTR char rtcBuffer[MAX_OFFLINE_RECORDS][MAX_LINE_LENGTH]; 
RTC_DATA_ATTR int bufferHead = 0;
RTC_DATA_ATTR int bufferTail = 0;
RTC_DATA_ATTR int rtcBufferCount = 0;
RTC_DATA_ATTR int sdFailCount = 0; 

// File size cache / Cache rozmiaru pliku
RTC_DATA_ATTR unsigned long currentFileSize = 0;
RTC_DATA_ATTR bool sizeSynced = false; 
RTC_DATA_ATTR int writesSinceSync = 0;

// Tracking filename in RTC / Śledzenie nazwy pliku w RTC
RTC_DATA_ATTR char lastFileName[35] = "";

// Wakeup counter / Licznik wybudzeń
RTC_DATA_ATTR uint32_t wakeCount = 0;

// SD Recovery counter / Licznik ratowania SD
RTC_DATA_ATTR uint32_t sdRecoveryCycle = 0;

// Stuck joystick counter / Licznik zablokowanego przycisku
// Incremented on entry, reset on normal measurement / Inkrementowany przy wejściu, zerowany przy pomiarze
RTC_DATA_ATTR uint8_t svcModeStreak = 0;

// RAM VARIABLES (Cleared on reset) / ZMIENNE RAM (Kasowane po resecie)
bool adsInitialized = false;
bool joyInitialized = false;
bool displayReady = false;
bool pmsUartReady = false;

// I2C mux error counter (Reset per wakeup) / Licznik błędów multipleksera
uint16_t muxErrCount = 0;

String configSSID = "";
String configPass = "";
int nh3Baseline = 0;
float nh3Factor = DEFAULT_NH3_FACTOR;

// Brownout register / Rejestr spadków napięcia
RTC_DATA_ATTR uint32_t brownout_reg_saved = 0;

// =================================================================================
// 1. SAFETY SYSTEMS (Watchdog & Brownout) / SYSTEMY BEZPIECZEŃSTWA
// =================================================================================

void initWatchdog() {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 45000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
  #else
    esp_task_wdt_init(45, true); 
    esp_task_wdt_add(NULL);
  #endif
}

// [HARDENING] tcaselect returns bool (detects MUX errors) / Zwraca bool w przypadku błędu MUX
bool tcaselect(uint8_t i) {
  if (i > 7) return false;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << i);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    muxErrCount++;
    Serial.print("[" FW_VERSION "] MUX ERR ch="); Serial.print(i);
    Serial.print(" code="); Serial.println(err);
    return false;
  }
  return true;
}

void brownoutSaveOnce() {
  if (brownout_reg_saved == 0) {
    brownout_reg_saved = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);
  }
}

void disableBrownout() {
  brownoutSaveOnce();
  CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
}

void enableBrownout() {
  if (brownout_reg_saved != 0) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brownout_reg_saved);
  } else {
    SET_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
  }
}

// OLED Helper
void ensureDisplay() {
  if (!displayReady) {
    tcaselect(CH_OLED);
    if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        displayReady = true;
    }
  }
}

// PMS UART Helper
void ensurePMSUart() {
    if (!pmsUartReady) {
        Serial2.begin(9600, SERIAL_8N1, PIN_PMS_RX, PIN_PMS_TX);
        pmsUartReady = true;
    }
}

// =================================================================================
// P-MOSFET POWER GATING (MH-Z19) / STEROWANIE ZASILANIEM
// =================================================================================

void powerMHZ19(bool on) {
  if (on) {
    Serial.println("[" FW_VERSION "] MH-Z19 POWER ON — start preheat");
    digitalWrite(PIN_MOSFET, LOW);
    
    delay(500);
    co2Serial.begin(9600, SERIAL_8N1, PIN_CO2_RX, PIN_CO2_TX);
    while (co2Serial.available()) co2Serial.read();
    myMHZ19.begin(co2Serial);
    myMHZ19.autoCalibration(false);
    
    unsigned long borrowedMs = (unsigned long)(FAN_DURATION_SEC + SETTLE_DURATION_SEC) * 1000UL;
    long extraWaitMs = (long)((unsigned long)MHZ19_PREHEAT_SEC * 1000UL) - (long)borrowedMs;
    
    if (extraWaitMs > 0) {
      Serial.print("[" FW_VERSION "] MH-Z19 extra preheat wait: ");
      Serial.print(extraWaitMs / 1000);
      Serial.println("s");
      
      unsigned long waitStart = millis();
      while (millis() - waitStart < (unsigned long)extraWaitMs) {
        delay(1000);
        esp_task_wdt_reset();
      }
    }
    
    Serial.println("[" FW_VERSION "] MH-Z19 preheat phase 1 done, continuing with fan+settle...");
    
  } else {
    Serial.println("[" FW_VERSION "] MH-Z19 POWER OFF");
    digitalWrite(PIN_MOSFET, HIGH);
    
    pinMode(PIN_CO2_TX, INPUT); 
    pinMode(PIN_CO2_RX, INPUT);
  }
}

// =================================================================================
// goToSleep() - DEEP SLEEP ROUTINE
// =================================================================================

void goToSleep() {
  Serial.print(">>> SLEEP " FW_VERSION " AK | wake#"); Serial.print(wakeCount);
  if (muxErrCount > 0) { Serial.print(" | MUX_ERR:"); Serial.print(muxErrCount); }
  Serial.println(" <<<");
  
  powerMHZ19(false);
  
  digitalWrite(PIN_FAN, LOW);
  
  ensurePMSUart();
  pms.sleep();

  if (displayReady) {
      // [HARDENING] Check if MUX is alive before sending OLED commands
      // Sprawdzamy czy mux żyje przed wysłaniem komend. Zapobiega to zużyciu +20mA.
      if (tcaselect(CH_OLED)) {
          display.clearDisplay(); 
          display.display();
          display.ssd1306_command(SSD1306_DISPLAYOFF);
      } else {
          Serial.println("[" FW_VERSION "] WARN: MUX FAIL — OLED charge pump may stay ON!");
      }
  }
  
  SD.end();
  
  pinMode(21, INPUT); pinMode(22, INPUT); pinMode(PIN_CS_SD, INPUT);
  
  Serial.flush();
  esp_deep_sleep_start();
}

// =================================================================================
// 2. SENSOR RESCUE PROTOCOLS / PROTOKOŁY RATUNKOWE CZUJNIKÓW
// =================================================================================

int getSafeCO2() {
  while (co2Serial.available()) co2Serial.read();
  esp_task_wdt_reset(); 

  int raw_co2 = myMHZ19.getCO2(); 
  esp_task_wdt_reset();
  
  if (raw_co2 > 0 && raw_co2 <= 10000) return raw_co2;

  Serial.print("ERR: CO2 Rescue...");
  myMHZ19.getTemperature(); 
  esp_task_wdt_reset();
  while (co2Serial.available()) co2Serial.read();
  esp_task_wdt_reset();

  raw_co2 = myMHZ19.getCO2();
  esp_task_wdt_reset();

  if (raw_co2 > 0 && raw_co2 <= 10000) return raw_co2;
  return -1;
}

// =================================================================================
// 3. FILE SYSTEM / SYSTEM PLIKÓW
// =================================================================================

bool initSD() {
  pinMode(PIN_CS_SD, OUTPUT); digitalWrite(PIN_CS_SD, HIGH);
  if (!SD.begin(PIN_CS_SD)) return false;
  return true;
}

bool writeToSD(const char* data, const char* fileName) {
  // SD Recovery
  if (sdFailCount > 5) {
    sdRecoveryCycle++;
    
    if (sdRecoveryCycle % SD_RECOVERY_INTERVAL == 0) {
      Serial.print("[" FW_VERSION "] SD Recovery attempt #");
      Serial.println(sdRecoveryCycle / SD_RECOVERY_INTERVAL);
      
      SD.end();
      enableBrownout();
      
      if (initSD()) {
        Serial.println("[" FW_VERSION "] SD Recovery SUCCESS!");
        sdFailCount = 0;
        sdRecoveryCycle = 0;
      } else {
        Serial.println("[" FW_VERSION "] SD Recovery FAILED");
        return false;
      }
    } else {
      return false;
    }
  }
  
  enableBrownout();

  if (!initSD()) return false;
  
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) { 
      f = SD.open(fileName, FILE_WRITE); 
      if (!f) return false; 
  }
  
  bool fileChanged = (strcmp(lastFileName, fileName) != 0);

  if (fileChanged) {
      currentFileSize = f.size();
      sizeSynced = true;
      writesSinceSync = 0;
      strncpy(lastFileName, fileName, sizeof(lastFileName)-1);
      lastFileName[sizeof(lastFileName)-1] = '\0';
  } else {
      if (!sizeSynced || writesSinceSync > 50) {
          currentFileSize = f.size();
          sizeSynced = true;
          writesSinceSync = 0;
      }
  }

  if (currentFileSize > MAX_FILE_SIZE_BYTES) {
    f.close();
    return false;
  }

  if (f.size() == 0) {
      f.println(CSV_HEADER);
      currentFileSize += strlen(CSV_HEADER) + 2;
  }

  if (f.println(data)) { 
      currentFileSize += strlen(data) + 2;
      writesSinceSync++;
      f.close(); 
      return true; 
  }
  
  f.close(); 
  return false;
}

void saveToOfflineBuffer(const char* data) {
  // [HARDENING] RTC memory corruption protection (brownout bit-flip)
  // Ochrona przed korupcją pamięci RTC (zapobiega crashom i zapisanu poza zakresem)
  if (bufferHead < 0 || bufferHead >= MAX_OFFLINE_RECORDS) bufferHead = 0;
  if (bufferTail < 0 || bufferTail >= MAX_OFFLINE_RECORDS) bufferTail = 0;
  if (rtcBufferCount < 0 || rtcBufferCount > MAX_OFFLINE_RECORDS) rtcBufferCount = 0;

  strncpy(rtcBuffer[bufferHead], data, MAX_LINE_LENGTH - 1);
  rtcBuffer[bufferHead][MAX_LINE_LENGTH - 1] = '\0';
  
  if (rtcBufferCount < MAX_OFFLINE_RECORDS) rtcBufferCount++;
  else bufferTail = (bufferTail + 1) % MAX_OFFLINE_RECORDS;
  
  bufferHead = (bufferHead + 1) % MAX_OFFLINE_RECORDS;
}

void flushBufferToSD() {
  // [HARDENING] RTC index sanity check / Sanityzacja indeksów RTC
  if (bufferHead < 0 || bufferHead >= MAX_OFFLINE_RECORDS) bufferHead = 0;
  if (bufferTail < 0 || bufferTail >= MAX_OFFLINE_RECORDS) bufferTail = 0;
  if (rtcBufferCount < 0 || rtcBufferCount > MAX_OFFLINE_RECORDS) rtcBufferCount = 0;
  
  if (rtcBufferCount == 0) return;
  
  enableBrownout();
  
  if (!initSD()) return;
  
  char fileName[35]; tcaselect(CH_RTC);
  if (rtc.begin()) { 
      DateTime now = rtc.now();
      snprintf(fileName, sizeof(fileName), "/pomiar_%04d-%02d-%02d.csv", now.year(), now.month(), now.day());
  } else strcpy(fileName, "/pomiar_unknown.csv");
  
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) f = SD.open(fileName, FILE_WRITE); 
  if (!f) return;
  
  strncpy(lastFileName, fileName, sizeof(lastFileName)-1);
  lastFileName[sizeof(lastFileName)-1] = '\0';
  
  currentFileSize = f.size();
  
  if (currentFileSize > MAX_FILE_SIZE_BYTES) { f.close(); return; } 

  if (f.size() == 0) {
      f.println(CSV_HEADER);
      currentFileSize += strlen(CSV_HEADER) + 2;
  }

  for (int i = 0; i < rtcBufferCount; i++) { 
      int idx = (bufferTail + i) % MAX_OFFLINE_RECORDS;
      f.println(rtcBuffer[idx]);
      currentFileSize += strlen(rtcBuffer[idx]) + 2; 
      esp_task_wdt_reset(); 
  }
  f.close();
  
  rtcBufferCount = 0; bufferHead = 0; bufferTail = 0; sdFailCount = 0;
  sdRecoveryCycle = 0;
  sizeSynced = true;
}

// =================================================================================
// 4. UI & SETUP / INTERFEJS UŻYTKOWNIKA
// =================================================================================

String inputKeyboard(String title, String initialValue = "") {
  ensureDisplay();

  String currentText = initialValue;
  const char chars[] = "abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_+-=., ";
  int charIndex = 0; int len = strlen(chars); bool editing = true;
  unsigned long lastMove = 0;
  unsigned long startTime = millis();  // UI Timeout
  int viewOffset = 0; const int maxCharsOnScreen = 16;

  while (editing) {
    esp_task_wdt_reset(); 
    
    // UI Timeout to prevent stuck loops / Timeout zapobiegający blokadzie
    if (millis() - startTime > UI_TIMEOUT_MS) {
      Serial.println("[" FW_VERSION "] inputKeyboard TIMEOUT");
      return currentText; 
    }
    
    tcaselect(CH_JOY); int x = joystick.getHorizontal(); int y = joystick.getVertical();
    bool btn = (joystick.getButton() == 0);
    
    tcaselect(CH_OLED);

    if (millis() - lastMove > 150) {
      if (y < 250) { charIndex--; if (charIndex < 0) charIndex = len - 1; lastMove = millis(); }
      if (y > 750) { charIndex++; if (charIndex >= len) charIndex = 0; lastMove = millis(); }
      if (x > 750) { 
        if (currentText.length() < 63) { 
          currentText += chars[charIndex]; 
          lastMove = millis() + 250;
          startTime = millis(); 
        } 
      }
      if (x < 250) { 
        if (currentText.length() > 0) { 
          currentText.remove(currentText.length() - 1); 
          lastMove = millis() + 250;
          startTime = millis(); 
        } 
      }
    }
    if (btn) { delay(300); return currentText; }

    // Text scrolling logic / Logika scrollowania tekstu
    int textLen = (int)currentText.length();
    if (textLen > maxCharsOnScreen) {
      viewOffset = textLen - maxCharsOnScreen;
    } else {
      viewOffset = 0;
    }

    display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0); display.print(title); display.setCursor(0, 15);
    if (viewOffset > 0) display.print("<");
    String visibleStr = currentText.substring(viewOffset, min(textLen, viewOffset + maxCharsOnScreen));
    display.print(visibleStr);
    int cursorRelX = visibleStr.length() + (viewOffset > 0 ? 1 : 0);
    display.fillRect(cursorRelX * 6, 15, 8, 10, WHITE);
    display.setCursor(cursorRelX * 6 + 1, 16); display.setTextColor(BLACK, WHITE); display.print(chars[charIndex]); display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 45);
    display.println("BTN: OK"); display.display();
  }
  return "";
}

String selectNetwork() {
  disableBrownout();
  
  ensureDisplay();
  display.clearDisplay(); display.setCursor(0,0); display.println("Skanowanie..."); display.display();
  
  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_task_wdt_reset(); 
  int n = WiFi.scanNetworks();
  esp_task_wdt_reset(); 
  
  enableBrownout();

  if (n == 0) { display.clearDisplay(); display.println("Brak sieci!"); display.display(); delay(2000); return ""; }
  
  int selected = 0; bool selecting = true; unsigned long lastMove = 0;
  unsigned long selectStart = millis(); 
  
  while (selecting) {
    esp_task_wdt_reset(); 
    
    if (millis() - selectStart > UI_TIMEOUT_MS) {
      Serial.println("[" FW_VERSION "] selectNetwork TIMEOUT");
      WiFi.scanDelete();
      return "";
    }
    
    tcaselect(CH_JOY); int y = joystick.getVertical(); bool btn = (joystick.getButton() == 0); tcaselect(CH_OLED);
    if (millis() - lastMove > 200) {
      if (y < 250) { selected--; if (selected < 0) selected = n - 1; lastMove = millis(); selectStart = millis(); }
      if (y > 750) { selected++; if (selected >= n) selected = 0; lastMove = millis(); selectStart = millis(); }
    }
    if (btn) { delay(300); return WiFi.SSID(selected); }
    display.clearDisplay(); display.setCursor(0,0); display.println("Wybierz siec:");
    int startIdx = max(0, min(selected, n - 3));
    for (int i = 0; i < 3 && (startIdx + i) < n; i++) {
      int current = startIdx + i;
      display.print(current == selected ? ">" : " "); display.println(WiFi.SSID(current).substring(0,14));
    }
    display.display();
  }
  return "";
}

void setNH3Factor() {
  String currentVal = String(nh3Factor, 5); 
  String input = inputKeyboard("Faktor (0.0-0.1)", currentVal);
  if (input.length() > 0) {
    float newFactor = input.toFloat();
    if (newFactor > 0.00001 && newFactor < 0.1) { 
      nh3Factor = newFactor;
      calibPref.begin("calib", false);
      calibPref.putFloat("nh3_factor", nh3Factor); calibPref.end();
      ensureDisplay();
      tcaselect(CH_OLED); display.clearDisplay(); display.setCursor(0, 20); display.println("ZAPISANO!"); display.print("Nowy F: "); display.println(nh3Factor, 5); display.display(); delay(2000);
    } else {
      ensureDisplay();
      tcaselect(CH_OLED); display.clearDisplay(); display.setCursor(0, 20); display.println("BLAD ZAKRESU!"); display.display(); delay(2000);
    }
  }
}

void calibrateNH3Zero() {
  ensureDisplay();
  tcaselect(CH_OLED); display.clearDisplay(); display.println("Kalib. NH3..."); display.display();
  tcaselect(CH_ADS); if (!adsInitialized) return;
  long sum = 0;
  for(int i=0; i<10; i++) { sum += ads.readADC_SingleEnded(0); delay(100); esp_task_wdt_reset(); }
  nh3Baseline = sum / 10;
  calibPref.begin("calib", false);
  calibPref.putInt("nh3_base", nh3Baseline); calibPref.end();
  tcaselect(CH_OLED); display.println("OK!"); display.display(); delay(1500);
}

void calibrateCO2() {
  ensureDisplay();
  tcaselect(CH_OLED); display.clearDisplay(); display.println("KALIBRACJA CO2\n400 ppm\n[BTN] Start"); display.display();
  
  tcaselect(CH_JOY);
  unsigned long btnTimeout = millis();
  while(joystick.getButton() != 0 && millis() - btnTimeout < 30000) { delay(10); esp_task_wdt_reset(); }
  if (millis() - btnTimeout >= 30000) return;
  
  btnTimeout = millis();
  while(joystick.getButton() == 0 && millis() - btnTimeout < 30000) { delay(10); esp_task_wdt_reset(); }
  if (millis() - btnTimeout >= 30000) return;
  
  powerMHZ19(true);
  
  tcaselect(CH_OLED); display.clearDisplay(); display.println("Wait..."); display.display();
  myMHZ19.calibrate(); 
  esp_task_wdt_reset();  
  delay(2000); 
  esp_task_wdt_reset();  
  display.println("GOTOWE!"); display.display(); delay(2000);
}

void wifiSync() {
  disableBrownout();
  
  ensureDisplay();
  display.clearDisplay(); display.println("Laczenie..."); display.display();
  WiFi.begin(configSSID.c_str(), configPass.c_str());
  int t=0;
  while(WiFi.status() != WL_CONNECTED && t<20) { delay(500); t++; esp_task_wdt_reset(); }
  
  if (WiFi.status() == WL_CONNECTED) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      // NTP validation to prevent 1970 clock wraparound / Walidacja chroniąca przed awarią NTP
      int ntpYear = timeinfo.tm_year + 1900;
      if (ntpYear >= 2024 && ntpYear <= 2099) {
        tcaselect(CH_RTC); 
        if(rtc.begin()) rtc.adjust(DateTime(ntpYear, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
        display.println("Czas OK");
      } else {
        Serial.print("[" FW_VERSION "] NTP rejected: year="); Serial.println(ntpYear);
        display.println("NTP: BAD YEAR!");
      }
    }
  } else display.println("Brak WiFi");
  display.display(); delay(2000); WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  
  enableBrownout();
}

// =================================================================================
// MAIN MEASUREMENT SEQUENCE / LOGIKA GŁÓWNA
// =================================================================================

void calculateAndSetWakeup();
void runServiceMode();

void runMeasurementSequence() {
  esp_task_wdt_reset(); 
  
  flushBufferToSD();
  
  powerMHZ19(true);
  
  digitalWrite(PIN_FAN, HIGH); delay(FAN_DURATION_SEC * 1000);
  digitalWrite(PIN_FAN, LOW);
  
  long wait = (SETTLE_DURATION_SEC - PMS_WARMUP_SEC) * 1000;
  unsigned long startWait = millis();
  while (millis() - startWait < (unsigned long)wait && wait > 0) { delay(100); esp_task_wdt_reset(); }

  ensurePMSUart(); 
  pms.wakeUp();
  
  startWait = millis();
  while (millis() - startWait < (unsigned long)(PMS_WARMUP_SEC * 1000)) { delay(100); esp_task_wdt_reset(); }

  float t = NAN, h = NAN;
  int16_t nh3_raw = 0; float nh3_ppm = 0.0;
  int pm1 = 0, pm25 = 0, pm10 = 0;
  char ts[25] = "RTC_ERR";
  
  int co2_ppm = getSafeCO2();
  
  if (tcaselect(CH_AHT)) {
    if (aht.begin()) { 
        sensors_event_t he, te;
        aht.getEvent(&he, &te);
        if (!isnan(te.temperature) && !isnan(he.relative_humidity)) { t = te.temperature; h = he.relative_humidity; }
    }
  } else {
    Serial.println("[" FW_VERSION "] MUX FAIL: AHT — temp/hum = NAN");
  }

  if (tcaselect(CH_ADS)) {
    if (adsInitialized) { 
        nh3_raw = ads.readADC_SingleEnded(0);
        if (nh3_raw < 32700 && nh3_raw > -1000) {
            int cal = nh3_raw - nh3Baseline;
            nh3_ppm = (cal < 0 ? 0 : (float)cal * nh3Factor);
        } else nh3_ppm = -1.0;
    }
  } else {
    Serial.println("[" FW_VERSION "] MUX FAIL: ADS — NH3 = -1");
    nh3_ppm = -1.0;  // Signal error in CSV
  }
  
  pms.requestRead();
  if (pms.readUntil(pmsData, 4000)) { pm1 = pmsData.PM_AE_UG_1_0; pm25 = pmsData.PM_AE_UG_2_5; pm10 = pmsData.PM_AE_UG_10_0; }
  pms.sleep();

  if (tcaselect(CH_RTC)) {
    char fileName[35] = "/pomiar_unknown.csv";
    if (rtc.begin()) {
      DateTime now = rtc.now();
      snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      snprintf(fileName, sizeof(fileName), "/pomiar_%04d-%02d-%02d.csv", now.year(), now.month(), now.day());
    }

    char line[MAX_LINE_LENGTH];
    snprintf(line, MAX_LINE_LENGTH, "%s,%.2f,%.2f,%d,%.3f,%d,%d,%d,%d", ts, t, h, nh3_raw, nh3_ppm, co2_ppm, pm1, pm25, pm10);
    
    if (writeToSD(line, fileName)) sdFailCount = 0;
    else { sdFailCount++; saveToOfflineBuffer(line); }
  } else {
    // MUX failure fallback / Ratowanie pomiaru przy awarii MUX
    Serial.println("[" FW_VERSION "] MUX FAIL: RTC — using fallback timestamp");
    char line[MAX_LINE_LENGTH];
    snprintf(line, MAX_LINE_LENGTH, "MUX_RTC_ERR,%.2f,%.2f,%d,%.3f,%d,%d,%d,%d", t, h, nh3_raw, nh3_ppm, co2_ppm, pm1, pm25, pm10);
    saveToOfflineBuffer(line);
  }

  powerMHZ19(false);

  unsigned long wStart = millis();
  
  ensureDisplay();
  
  // [HARDENING] Single MUX test prevents I2C timeout blocking / Test zapobiegający timeoutom
  bool muxAlive = tcaselect(CH_OLED);
  
  if (!muxAlive) {
    // MUX dead -> Blind wait / MUX martwy -> Pasywne czekanie
    Serial.println("[" FW_VERSION "] MUX dead in display loop — blind wait");
    while (millis() - wStart < DISPLAY_RESULT_MS) {
      delay(100);
      esp_task_wdt_reset();
    }
  } else {
    while (millis() - wStart < DISPLAY_RESULT_MS) {
      esp_task_wdt_reset(); 
      tcaselect(CH_JOY);
      // Stuck joystick loop check / Sprawdzanie w pętli display
      if (joystick.getButton() == 0 && svcModeStreak <= SVC_MODE_STREAK_MAX) { 
        delay(1000); 
        if(joystick.getButton() == 0) { 
          svcModeStreak++;  
          runServiceMode(); 
          return; 
        } 
      }
      tcaselect(CH_OLED);
      display.clearDisplay(); display.setCursor(0,0);
      display.println("ZAPISANO. CO2:"); display.println(co2_ppm);
      
      if (sdFailCount > 5) display.println("! SD RECOVERY !");
      else if (sdFailCount > 0) display.println("! SD ERROR !");
      else display.println("SD OK");
      
      // I2C errors on screen / Sygnalizacja na ekranie
      if (muxErrCount > 0) {
        display.print("! MUX ERR:"); display.print(muxErrCount); display.println(" !");
      }
      
      display.println("\nMENU? Przytrzymaj BTN");
      display.fillRect(0, 60, map(millis()-wStart, 0, DISPLAY_RESULT_MS, 0, 128), 4, WHITE);
      display.display(); delay(100);
    }
  }
  calculateAndSetWakeup();
}

// =================================================================================
// runServiceMode() - USER MENU / MENU SERWISOWE
// =================================================================================

void runServiceMode() {
  ensureDisplay();
  
  int screen = 0; bool active = true; unsigned long lastAct = millis();
  while (active) {
    esp_task_wdt_reset();
    tcaselect(CH_JOY); int y = joystick.getVertical(); bool btn = (joystick.getButton() == 0);
    static unsigned long lastMove = 0;
    if (millis()-lastMove > 250) {
      if (y < 250) { screen--; lastMove=millis(); lastAct=millis(); }
      if (y > 750) { screen++; lastMove=millis(); lastAct=millis(); }
      if (screen < 0) screen = 2; if (screen > 2) screen = 0;
    }

    tcaselect(CH_OLED); display.clearDisplay(); display.setCursor(0,0);
    if (screen==0) {
      display.print("[ STATUS "); display.print(FW_VERSION); display.println(" AK ]");
      
      display.print("SD: ");
      if (sdFailCount > 5) {
        display.print("RECOVERY ("); display.print(sdRecoveryCycle); display.println(")");
      } else if (sdFailCount > 0) {
        display.print("ERR "); display.println(sdFailCount);
      } else {
        display.println("OK");
      }
      
      display.print("Buf: "); display.print(rtcBufferCount);
      display.print(" FSz: "); display.print(currentFileSize / 1024); display.println("KB");
      display.print("Wake: "); display.print(wakeCount);
      
      // Diagnostics / Diagnostyka
      if (muxErrCount > 0) { display.print(" MX:"); display.print(muxErrCount); }
      display.println();
      
      if (svcModeStreak > SVC_MODE_STREAK_MAX) {
        display.println("JOY: STUCK!");
      }
    } else if (screen==1) {
      display.println("[ LIVE ]");
      if (tcaselect(CH_ADS)) {
        int r = ads.readADC_SingleEnded(0);
        float n_ppm = (float)(r - nh3Baseline < 0 ? 0 : r - nh3Baseline) * nh3Factor;
        tcaselect(CH_OLED); display.print("NH3: "); display.print(n_ppm, 3); display.println(" ppm");
      } else {
        tcaselect(CH_OLED); display.println("NH3: MUX ERR");
      }
      
      int c_ppm = getSafeCO2();
      tcaselect(CH_OLED);
      display.print("CO2: ");
      if (c_ppm > 0) { display.print(c_ppm); display.println(" ppm"); }
      else display.println("OFF/ERR");
    } else if (screen==2) display.println("[ MENU ]\n[KLIK] Wejdz");
    display.display();

    if (btn && millis()-lastAct > 500) {
      lastAct = millis();
      if (screen==2) {
        int mIdx = 0; bool sMenu = true;
        const int menuCount = 6; 
        const char* menuItems[] = { "Sync Czas", "WiFi Setup", "Kalib. NH3 (Zero)", "Kalib. CO2", "Ustaw Faktor NH3", "Wyjscie" };
        while(sMenu) {
          esp_task_wdt_reset();
          tcaselect(CH_JOY); int sy = joystick.getVertical();
          if (millis() - lastMove > 200) {
            if (sy < 250) { mIdx--; if(mIdx < 0) mIdx = menuCount - 1; lastMove = millis(); }
            if (sy > 750) { mIdx++; if(mIdx >= menuCount) mIdx = 0; lastMove = millis(); }
          }
          tcaselect(CH_OLED);
          display.clearDisplay(); display.setCursor(0,0); display.println("[ MENU GLOWNE ]");
          int startList = 0; if (mIdx > 3) startList = mIdx - 3;
          for (int i = 0; i < 4; i++) {
             int itemIndex = startList + i;
             if (itemIndex < menuCount) {
               display.setCursor(5, 14 + (i * 12));
               display.print(mIdx == itemIndex ? "> " : "  "); display.println(menuItems[itemIndex]);
             }
          }
          int scrollBarY = map(mIdx, 0, menuCount-1, 14, 50);
          display.fillRect(124, scrollBarY, 3, 3, WHITE);
          display.display();
          if (joystick.getButton() == 0) {
            delay(200);
            if(mIdx == 0) wifiSync();
            if(mIdx == 1) { String s = selectNetwork(); if(s!="") { String p = inputKeyboard("Haslo WiFi"); pref.begin("wifi",false); pref.putString("ssid",s); pref.putString("pass",p); pref.end(); } }
            if(mIdx == 2) calibrateNH3Zero();
            if(mIdx == 3) calibrateCO2();
            if(mIdx == 4) setNH3Factor();
            if(mIdx == 5) sMenu = false;
            lastAct = millis();
            display.clearDisplay(); display.display();
          }
          if (millis() - lastAct > 30000) sMenu = false;
        }
      }
    }
    if (millis()-lastAct > 30000) active = false;
    delay(50);
  }
  calculateAndSetWakeup();
}

void calculateAndSetWakeup() {
  tcaselect(CH_RTC); if (!rtc.begin()) { esp_sleep_enable_timer_wakeup(15 * 60 * 1000000ULL); goToSleep(); }
  DateTime now = rtc.now();
  long cur = now.minute()*60 + now.second();
  long tgt = (cur < 900) ? 900 : (cur < 1800 ? 1800 : (cur < 2700 ? 2700 : 3600));
  long sleep = (tgt - cur) - WAKEUP_OFFSET_SEC;
  if (sleep < 10) sleep += 900;
  esp_sleep_enable_timer_wakeup((unsigned long long)sleep * 1000000ULL);
  goToSleep();
}

void setup() {
  Serial.begin(115200);
  initWatchdog(); 
  
  wakeCount++;
  Serial.print("[" FW_VERSION "] Wake #"); Serial.println(wakeCount);

  pinMode(PIN_FAN, OUTPUT); digitalWrite(PIN_FAN, LOW);
  
  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, HIGH);  // MH-Z19 OFF
  
  Wire.begin(21, 22); Wire.setTimeOut(50);
  
  tcaselect(CH_JOY); joyInitialized = joystick.begin();
  tcaselect(CH_ADS); if(ads.begin()) { ads.setGain(GAIN_ONE); adsInitialized = true; }
  
  calibPref.begin("calib", true); 
  nh3Baseline = calibPref.getInt("nh3_base", 0); 
  nh3Factor = calibPref.getFloat("nh3_factor", DEFAULT_NH3_FACTOR); 
  calibPref.end();

  pref.begin("wifi", true);
  configSSID = pref.getString("ssid", ""); configPass = pref.getString("pass", ""); pref.end();

  brownoutSaveOnce();

  // [HARDENING] Stuck joystick — full logic / Pełna logika zapobiegająca zawieszaniu
  // Streak reset ONLY if button is physically free / Zerowany tylko jeśli przycisk wolny
  if (joyInitialized) {
    tcaselect(CH_JOY);
    if (joystick.getButton() == 0) {
      // Button pressed on wakeup / Przycisk wciśnięty przy wybudzeniu
      svcModeStreak++;
      if (svcModeStreak <= SVC_MODE_STREAK_MAX) {
        Serial.print("[" FW_VERSION "] ServiceMode entry (streak: "); 
        Serial.print(svcModeStreak); Serial.println(")");
        runServiceMode();  // -> calculateAndSetWakeup() -> sleep
      } else {
        Serial.print("[" FW_VERSION "] BTN STUCK — ignoring (streak: ");
        Serial.print(svcModeStreak); Serial.println(")");
        // Fall through to runMeasurementSequence() / Ignoruj i rób pomiar
      }
    } else {
      // Button FREE — reset streak / Przycisk WOLNY
      if (svcModeStreak > 0) {
        Serial.print("[" FW_VERSION "] BTN released, streak reset (was: ");
        Serial.print(svcModeStreak); Serial.println(")");
      }
      svcModeStreak = 0;
    }
  }
  
  runMeasurementSequence();
}

void loop() {}