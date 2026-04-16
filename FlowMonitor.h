#ifndef FLOW_MONITOR_H
#define FLOW_MONITOR_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>
#include <EEPROM.h>
#include "RTClib.h"
#include "SdFat.h"

// I2C0 (OLED, RTC)
#define I2C0_SDA_PIN 8
#define I2C0_SCL_PIN 9

// SD SPI
#define SD_CS_PIN   5
#define SD_SCK_PIN  12
#define SD_MISO_PIN 13
#define SD_MOSI_PIN 11

// O2 analogue
#define O2_ADC_PIN  4

// Buzzer
#define ALARM_BUZZER_PIN 21

// Keypad
const byte ROWS = 4;
const byte COLS = 4;
extern char keys[ROWS][COLS];
extern byte rowPins[ROWS];
extern byte colPins[COLS];

// Serial communication
#define SERIAL_FROM_A Serial2
#define RX_FROM_A_PIN 16   
#define TX_TO_A_PIN   17   

// ==================== System constant ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define EEPROM_SIZE 512
#define CALIB_ZERO_ADDR 0
#define CALIB_SPAN_ADDR 4
#define CALIB_SENS_ADDR 8
#define FLOW_CALIB_ADDR 12
#define LAST_RECORD_ADDR 100
#define SFM3300_CALIB_ADDR 50 

#define SAMPLE_INTERVAL 1000
#define LOG_INTERVAL 60000
#define BACKUP_INTERVAL 300000
#define MAX_RETRY_ATTEMPTS 3
#define I2C_TIMEOUT_MS 100
#define MAX_RECORD_SIZE 1000

// Thresholds
const float O2_LOW_ALARM = 19.5;
const float O2_HIGH_ALARM = 95.0;
const float FLOW_LOW_ALARM = 0.1;
const float FLOW_HIGH_ALARM = 15.0;

#define FLOW_TIMEOUT_MS 3000

// ==================== Struct ====================
struct CalibrationData {
  float zero_mV;
  float span_mV;
  float span_o2;
  float sensitivity;
  float flow_offset;
  float flow_scale;
  float sfm3300_offset;  // unvailable
  float sfm3300_scale;
  uint32_t calib_time;
  uint8_t crc8;
};

struct SensorData {
  float o2_percent;
  float flow_rate;
  float temperature;
  float pressure;
  uint32_t timestamp;
  uint8_t status;
  uint8_t crc8;
};

struct EventLog {
  uint32_t timestamp;
  uint8_t event_type;
  char description[64];
  uint8_t crc8;
};

// ==================== Global variable ====================
extern Adafruit_SSD1306 display;
extern Keypad keypad;
extern RTC_DS3231 rtc;

extern SdFs sd;
extern FsFile dataFile;
extern FsFile logFile;
extern FsVolume *vol;

extern CalibrationData calib;
extern SensorData current_data;
extern SensorData backup_data;

extern bool sd_available;
extern bool rtc_available;
extern bool flow_sensor_online;   
extern uint32_t last_sample;
extern uint32_t last_log;
extern uint32_t last_backup;
extern uint32_t last_keypress;
extern bool alarm_active;
extern char alarm_message[50];
extern uint16_t record_count;
extern uint16_t file_record_count;
extern uint8_t display_mode;

extern float last_flow_value;
extern uint32_t last_flow_receive_time;

// ==================== Functions ====================
// Initialization
void initializeSystem();
void initializeSensors();
bool initializeSDCard();
void initializeRTC();

// O2 data
void readO2Sensor();
float readAD620Voltage(int samples = 10);

// Flow data
void parseSerialData();
void readFlowSensor();   // 从 last_flow_value 获取

// Calibration
bool loadCalibration();
bool saveCalibration();
void performCalibration();
void performValidation();

// Display
void updateDisplay();
void showMainScreen();
void showDetailsScreen();
void showLogScreen();
void showCalibrationScreen();
void showValidationScreen();
void showMainMenu();
void showEventLog();
void showSettingsMenu();

// Keypad
void handleKeypad();
void processMenu(char key);

// Logging
bool saveDataToSD();
bool logEvent(uint8_t type, const char* desc);
void checkFileRotation();

// Alarm
void checkAlarms();

// Backup
bool backupData();
bool restoreData();


void beep(int duration = 100);
DateTime getCurrentTime();
uint8_t calculateCRC8(const uint8_t *data, size_t length);
bool crcCheck(const uint8_t *data, size_t length, uint8_t crc);

#endif