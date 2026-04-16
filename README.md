Medical Oxygen & Flow Monitoring System

A low-cost embedded system for real-time monitoring of oxygen concentration and flow rate, designed for resource-limited healthcare settings (LMICs).

---

1 Overview

This project implements a dual-sensor monitoring system that measures:

- Oxygen concentration (%) via analogue sensor + ADC
- Flow rate (L/min) via I2C and serial communication (external flow sensor)

The system provides:

- Real-time display (OLED)
- Alarm detection (threshold-based)
- Data logging (SD card, CSV format)
- Event logging (SD + EEPROM)
- Backup & restore (EEPROM)
- User interaction via keypad

---

2 Hardware Components

- Microcontroller: ESP32-S3, ESP32
- O₂ Sensor (analogue output)
- Flow Sensor (I2C/UART output)
- OLED Display (SSD1306, I2C)
- RTC Module (DS3231)
- SD Card Module (SPI)
- Keypad (4x4)

---

3 Pin Configuration 

| Component | Interface | Pins |

| OLED| I2C | SDA=8, SCL=9 |
| RTC | I2C | SDA=8, SCL=9 |
| SD Card | SPI | MISO=13, MOSI=11, SCK=12, CS=5 |
| Flow Sensor | I2C | SDA=21, SCL=22 | UART | RX=16，TX=17 |
| O₂ Sensor | ADC | GPIO4 |

---

4 Calibration

The O₂ sensor is calibrated using:

- Zero calibration (0%)
- Span calibration (known concentration, e.g. 20.9% or 25%)

---

5 Data Logging

Data is stored in CSV format: timestamp,o2_percent,flow_rate,temp,flow_online

-Location: /DATA/data_YYYYMMDD.csv (data)
-Location: /LOGS/events.csv (event)

---

This system is a prototype for research purposes only and is not certified for clinical use.

---
