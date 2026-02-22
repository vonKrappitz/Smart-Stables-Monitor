\# 🔌 Smart Stables Monitor v1.0.0 - Wiring Guide / Schemat Połączeń



🇬🇧 \*\*\[English Version below]\*\* | 🇵🇱 \*\*\[Wersja Polska poniżej]\*\*



---



\## 🇬🇧 English Version



\### 📌 ESP32 Pinout Mapping

| ESP32 Pin | Component / Function | Notes |

| :--- | :--- | :--- |

| \*\*GPIO 21\*\* | \[cite\_start]I2C SDA \[cite: 238] | Main I2C Bus to Multiplexer |

| \*\*GPIO 22\*\* | \[cite\_start]I2C SCL \[cite: 238] | Main I2C Bus to Multiplexer |

| \*\*GPIO 5\*\* | \[cite\_start]SD Card CS (Chip Select) \[cite: 34] | Standard VSPI uses 18(SCK), 19(MISO), 23(MOSI) |

| \*\*GPIO 4\*\* | \[cite\_start]Fan Control `PIN\_FAN` \[cite: 34] | Triggers the air sampling fan |

| \*\*GPIO 16\*\* | \[cite\_start]PMS Sensor RX `PIN\_PMS\_RX` \[cite: 35] | UART2 RX (Connect to PMS TX) |

| \*\*GPIO 17\*\* | \[cite\_start]PMS Sensor TX `PIN\_PMS\_TX` \[cite: 35] | UART2 TX (Connect to PMS RX) |

| \*\*GPIO 32\*\* | \[cite\_start]MH-Z19 CO2 RX `PIN\_CO2\_RX` \[cite: 35] | UART1 RX (Connect to MH-Z19 TX) |

| \*\*GPIO 33\*\* | \[cite\_start]MH-Z19 CO2 TX `PIN\_CO2\_TX` \[cite: 35] | UART1 TX (Connect to MH-Z19 RX) |

| \*\*GPIO 13\*\* | \[cite\_start]MH-Z19 Power MOSFET Gate \[cite: 32] | Controls P-Channel MOSFET (IRLML6401) |



\### 🔀 I2C Multiplexer (TCA9548A - Address 0x70)

\[cite\_start]Due to address conflicts and bus stability, all I2C devices are routed through an 8-channel multiplexer at address `0x70`\[cite: 36].



| MUX Channel | Component Attached | Notes |

| :--- | :--- | :--- |

| \*\*CH 0\*\* | \[cite\_start]SSD1306 OLED Display \[cite: 36] | \[cite\_start]Address usually `0x3C` \[cite: 55] |

| \*\*CH 1\*\* | \[cite\_start]AHT10 / AHT20 Sensor \[cite: 36] | Temperature \& Humidity |

| \*\*CH 2\*\* | \[cite\_start]DS3231 RTC Module \[cite: 36] | Real-Time Clock |

| \*\*CH 3\*\* | \[cite\_start]ADS1115 ADC \[cite: 36] | \[cite\_start]Reads Analog NH3 Sensor on ADC Channel 0 \[cite: 142] |

| \*\*CH 4\*\* | \[cite\_start]Qwiic Joystick \[cite: 36] | User Interface control |



\### ⚡ Power Gating (MH-Z19 CO2 Sensor)

\[cite\_start]To conserve power, the MH-Z19 sensor is dynamically powered via a P-Channel MOSFET (e.g., IRLML6401) controlled by \*\*GPIO 13\*\*\[cite: 32]:

\* \[cite\_start]\*\*GPIO 13 LOW\*\*: MH-Z19 is \*\*ON\*\* (Preheat/Measure mode)\[cite: 33].

\* \[cite\_start]\*\*GPIO 13 HIGH\*\*: MH-Z19 is \*\*OFF\*\* (Deep Sleep mode)\[cite: 33].



---



\## 🇵🇱 Wersja Polska



\### 📌 Przypisanie Pinów ESP32

| Pin ESP32 | Komponent / Funkcja | Uwagi |

| :--- | :--- | :--- |

| \*\*GPIO 21\*\* | \[cite\_start]I2C SDA \[cite: 238] | Główna szyna I2C (do Multipleksera) |

| \*\*GPIO 22\*\* | \[cite\_start]I2C SCL \[cite: 238] | Główna szyna I2C (do Multipleksera) |

| \*\*GPIO 5\*\* | \[cite\_start]SD Card CS (Chip Select) \[cite: 34] | Piny SPI domyślnie: 18(SCK), 19(MISO), 23(MOSI) |

| \*\*GPIO 4\*\* | \[cite\_start]Wentylator `PIN\_FAN` \[cite: 34] | Sterowanie wentylatorem próbkującym powietrze |

| \*\*GPIO 16\*\* | \[cite\_start]PMS Sensor RX `PIN\_PMS\_RX` \[cite: 35] | UART2 RX (Podłącz do TX czujnika PMS) |

| \*\*GPIO 17\*\* | \[cite\_start]PMS Sensor TX `PIN\_PMS\_TX` \[cite: 35] | UART2 TX (Podłącz do RX czujnika PMS) |

| \*\*GPIO 32\*\* | \[cite\_start]MH-Z19 CO2 RX `PIN\_CO2\_RX` \[cite: 35] | UART1 RX (Podłącz do TX czujnika CO2) |

| \*\*GPIO 33\*\* | \[cite\_start]MH-Z19 CO2 TX `PIN\_CO2\_TX` \[cite: 35] | UART1 TX (Podłącz do RX czujnika CO2) |

| \*\*GPIO 13\*\* | \[cite\_start]Bramka MOSFET (Zasilanie MH-Z19) \[cite: 32] | Sterowanie tranzystorem P-MOSFET (IRLML6401) |



\### 🔀 Multiplekser I2C (TCA9548A - Adres 0x70)

\[cite\_start]Ze względu na konflikty adresów oraz w celu izolacji magistrali, wszystkie urządzenia I2C są podłączone przez 8-kanałowy multiplekser znajdujący się pod adresem `0x70`\[cite: 36].



| Kanał MUX | Podłączony Komponent | Uwagi |

| :--- | :--- | :--- |

| \*\*CH 0\*\* | \[cite\_start]Ekran OLED SSD1306 \[cite: 36] | \[cite\_start]Adres domyślny `0x3C` \[cite: 55] |

| \*\*CH 1\*\* | \[cite\_start]Czujnik AHT10 / AHT20 \[cite: 36] | Temperatura i Wilgotność |

| \*\*CH 2\*\* | \[cite\_start]Moduł RTC DS3231 \[cite: 36] | Zegar Czasu Rzeczywistego |

| \*\*CH 3\*\* | \[cite\_start]Przetwornik ADS1115 \[cite: 36] | \[cite\_start]Odczytuje analogowy czujnik NH3 na Kanale 0 \[cite: 142] |

| \*\*CH 4\*\* | \[cite\_start]Qwiic Joystick \[cite: 36] | Nawigacja po menu urządzenia |



\### ⚡ Zarządzanie Zasilaniem (Czujnik CO2 MH-Z19)

\[cite\_start]Aby oszczędzać energię akumulatora, moduł MH-Z19 jest dynamicznie włączany i wyłączany za pomocą tranzystora P-MOSFET (np. IRLML6401), którym steruje \*\*GPIO 13\*\*\[cite: 32]:

\* \[cite\_start]\*\*GPIO 13 STAN NISKI (LOW)\*\*: MH-Z19 \*\*WŁĄCZONY\*\* (Rozgrzewanie/Pomiar)\[cite: 33].

\* \[cite\_start]\*\*GPIO 13 STAN WYSOKI (HIGH)\*\*: MH-Z19 \*\*WYŁĄCZONY\*\* (Tryb Deep Sleep)\[cite: 33].



> \*\*Uwaga sprzętowa:\*\* ESP32 operuje na logice 3.3V. Upewnij się, że zasilanie modułów (szczególnie czujnika pyłów PMS i MH-Z19) jest odpowiednio dostosowane (często wymagają 5V do zasilania grzałki/lasera, ale na liniach danych RX/TX należy zachować poziomy tolerowane przez ESP32).

