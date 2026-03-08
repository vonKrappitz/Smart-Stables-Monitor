# 🐴 Smart Stables Monitor v1.0.0 Stable

🇬🇧 **[English Version below](#en)** | 🇵🇱 **[Wersja Polska poniżej](#pl)**

<a id="en"></a>
## 🇬🇧 English Version

**Author:** Maciej Kasperek (vonKrappitz)

**Firmware Version:** v1.0.0 Stable

**Date:** 2026-02-22

### 📋 About the Project

**Smart Stables Monitor** is an advanced, ultra-reliable IoT device designed for continuous air quality monitoring in horse stables. Because environmental conditions in stables are demanding, this project focuses heavily on **defensive programming, fault tolerance, and hardware hardening**. It tracks crucial parameters for animal welfare: Ammonia (NH3), Carbon Dioxide (CO2), Particulate Matter (PM1, PM2.5, PM10), Temperature, and Humidity.

### 🛡️ Defensive Engineering & Hardening (v1.0.0 Stable)

This firmware is built to survive hardware failures, brownouts, and sensor degradation:

* **Defensive RTC Memory Protection:** Prevents cyclic buffer index corruption (bit-flips) during brownouts, ensuring the ESP32 doesn't crash from out-of-bounds memory access.
* **Stuck-Joystick Anti-Lock Logic:** Accidental or permanent mechanical locking of the control joystick is detected; after 3 failed service mode entries, the UI ignores the hardware fault and continues critical air quality measurements.
* **MUX-Safe I2C Loop & Sleep:** Continuous I2C multiplexer health checks. If the MUX dies, the system avoids 2.5s timeout loops, safely goes into deep sleep, and turns off the OLED charge pump to prevent a 20mA parasitic drain.
* **Offline Buffering & SD Recovery:** Automatically caches up to 10 offline records in RTC RAM if the SD card fails, attempting a full SPI recovery every 10 wakeup cycles.

### ⚙️ Hardware & Sensors

* **Microcontroller:** ESP32 (Watchdog enabled, Brownout-RMW logic).
* **Gas Sensors:** MH-Z19 (CO2, controlled via P-MOSFET IRLML6401), Analog NH3 Sensor via ADS1115 ADC.
* **Particulates & Climate:** PMS (PM1.0/2.5/10), AHT10/20 (Temp/Hum).
* **Peripherals:** DS3231 RTC, SSD1306 OLED, Qwiic Joystick, MicroSD module.
* **Bus Management:** 8-Channel I2C Multiplexer (0x70).

### 🔋 Power Profiling & Energy Balance

The system is optimized for long-term battery operation using deep sleep and precise sensor preheating sequences.

| State | Daily Duration | Current Draw | Daily Consumption |
| :--- | :--- | :--- | :--- |
| **Active Mode** | 4.4 h/day | 145 mA | 642 mAh/day |
| **Deep Sleep** | 19.6 h/day | 12 mA | 235 mAh/day |
| **Total Daily** | 24 h/day | - | **877 mAh/day** |

**Estimated Battery Life:** ~34 days on a 30,000 mAh battery pack.

### ⚖️ License

This project is licensed under a **Custom Non-Commercial License**.

It is free for personal, educational, and hobbyist use. **Commercial use, redistribution, or derivation without written consent is strictly prohibited.** The software is provided "AS IS" with a full disclaimer of liability regarding animal health, sensor accuracy, and hardware reliability. See the LICENSE file for details.

---

<a id="pl"></a>
## 🇵🇱 Wersja Polska

**Autor:** Maciej Kasperek (vonKrappitz) 

**Wersja Firmware:** v1.0.0 Stable

**Data:** 2026-02-22

### 📋 O Projekcie

**Smart Stables Monitor** to zaawansowane, wysoce niezawodne urządzenie IoT przeznaczone do ciągłego monitorowania jakości powietrza w stajniach. Ze względu na trudne warunki środowiskowe, główny nacisk w kodzie położono na **programowanie defensywne, tolerancję na błędy i hardening sprzętowy**. System śledzi kluczowe dla zdrowia koni parametry: Amoniak (NH3), Dwutlenek Węgla (CO2), Pyły zawieszone (PM1, PM2.5, PM10) oraz Temperaturę i Wilgotność.

### 🛡️ Niezawodność i "Hardening" (Nowości w v1.0.0 Stable)

Oprogramowanie zostało zaprojektowane tak, aby przetrwać awarie sprzętowe, spadki napięcia i degradację czujników:

* **Defensywna ochrona pamięci RTC:** Zapobiega korupcji indeksów bufora (tzw. bit-flips) podczas spadków napięcia (brownout), eliminując ryzyko crashu ESP32 z powodu zapisu poza tablicą.
* **Ochrona przed zablokowanym joystickiem:** Wykrywa mechaniczne zablokowanie przycisku. Po 3 nieudanych cyklach wejścia w tryb serwisowy, UI ignoruje błąd i bezwarunkowo kontynuuje pomiary.
* **MUX-Safe (Bezpieczny I2C):** Ciągłe monitorowanie stanu multipleksera I2C. Jeśli MUX ulegnie awarii, system unika opóźnień (timeoutów), wyłącza pompę ładunkową OLED (oszczędzając 20mA w trybie uśpienia) i bezpiecznie przechodzi w Deep Sleep.
* **Bufor Offline i Odzyskiwanie SD:** Automatycznie zapisuje do 10 pomiarów w pamięci RTC RAM w razie awarii karty SD, próbując wykonać pełny restart magistrali SPI co 10 wybudzeń.

### ⚙️ Hardware i Czujniki

* **Mikrokontroler:** ESP32 (Aktywny Watchdog, obsługa rejestrów Brownout).
* **Czujniki gazów:** MH-Z19 (CO2, zasilanie odcinane przez P-MOSFET IRLML6401), Analogowy czujnik NH3 przez przetwornik ADS1115.
* **Pyły i Klimat:** PMS (PM1.0/2.5/10), AHT10/20 (Temp/Hum).
* **Peryferia:** Moduł RTC DS3231, Ekran OLED SSD1306, Qwiic Joystick, Moduł MicroSD.
* **Zarządzanie Magistralą:** 8-kanałowy multiplekser I2C (0x70).

### 🔋 Profil Energetyczny i Bateria

System jest zoptymalizowany pod kątem długotrwałej pracy, wykorzystując precyzyjnie odmierzone czasy rozgrzewania czujników oraz głębokie uśpienie (Deep Sleep).

| Stan | Czas trwania / doba | Pobór prądu | Zużycie / doba |
| :--- | :--- | :--- | :--- |
| **Tryb Aktywny** | 4.4 h/d | 145 mA | 642 mAh/d |
| **Deep Sleep** | 19.6 h/d | 12 mA | 235 mAh/d |
| **Razem** | 24 h/d | - | **877 mAh/d** |

**Szacowany czas pracy:** ~34 dni na powerbanku / akumulatorze 30 000 mAh.

### ⚖️ Licencja

Ten projekt jest udostępniany na **Autorskiej Licencji Niekomercyjnej**.

Korzystanie w celach prywatnych, edukacyjnych i hobbystycznych jest całkowicie darmowe. **Wykorzystanie komercyjne, modyfikacja w celach zarobkowych lub sprzedaż bez pisemnej zgody autora jest surowo zabroniona.** Oprogramowanie dostarczane jest w modelu "AS IS" z pełnym wyłączeniem odpowiedzialności prawnej (m.in. za zdrowie zwierząt czy dokładność sensorów). Szczegóły w pliku LICENSE.

---
*Created by Maciej Kasperek (vonKrappitz)*
