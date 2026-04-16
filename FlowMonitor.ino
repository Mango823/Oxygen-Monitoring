#include "FlowMonitor.h"


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
RTC_DS3231 rtc;

SdFs sd;
FsFile dataFile;
FsFile logFile;
FsVolume *vol = nullptr;

// ==================== Global variable ====================
CalibrationData calib;
SensorData current_data;
SensorData backup_data;

bool sd_available = false;
bool rtc_available = false;
bool flow_sensor_online = false;   // flow sens
uint32_t last_sample = 0;
uint32_t last_log = 0;
uint32_t last_backup = 0;
uint32_t last_keypress = 0;
bool alarm_active = false;
char alarm_message[50] = "";
uint16_t record_count = 0;
uint16_t file_record_count = 0;
uint8_t display_mode = 0;           // 0:main, 1:info, 2:log

float last_flow_value = 0.0;
uint32_t last_flow_receive_time = 0;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {6, 7, 14, 15};
byte colPins[COLS] = {36, 37, 38, 39};

// ==================== CRC ====================
uint8_t calculateCRC8(const uint8_t *data, size_t length) {
  uint8_t crc = 0x00;
  uint8_t polynomial = 0x31;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ polynomial;
      else crc <<= 1;
    }
  }
  return crc;
}

bool crcCheck(const uint8_t *data, size_t length, uint8_t crc) {
  return calculateCRC8(data, length) == crc;
}

void beep(int duration) {
  tone(ALARM_BUZZER_PIN, 2000, duration);
}

DateTime getCurrentTime() {
  if (rtc_available) {
    return rtc.now();
  } else {
    static uint32_t start_time = 0;
    if (start_time == 0) start_time = millis();
    return DateTime(2024, 1, 1, 0, 0, 0 + (millis() - start_time) / 1000);
  }
}

// RTC temp
float readRTCTemperature() {
  if (!rtc_available) return -999.0;
  Wire.beginTransmission(0x68);
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2);
  if (Wire.available() >= 2) {
    uint8_t temp_msb = Wire.read();
    uint8_t temp_lsb = Wire.read();
    int16_t raw_temp = (temp_msb << 8) | temp_lsb;
    return raw_temp / 256.0;
  }
  return -999.0;
}

// Set RTC time
void setRTCtime() {
  if (!rtc_available) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("RTC not available!");
    display.display();
    delay(1500);
    return;
  }

  int year, month, day, hour, minute, second;
  bool input_cancelled = false;

  // Get current RTC time
  DateTime now = rtc.now();
  year = now.year();
  month = now.month();
  day = now.day();
  hour = now.hour();
  minute = now.minute();
  second = now.second();

  // Read 2 num
  auto readTwoDigits = [&](const char* prompt, int &value, int min_val, int max_val) -> bool {
    char buf[3] = {0,0,0};
    int idx = 0;
    while (true) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.printf("%s (0-9):\n", prompt);
      display.printf("%s", buf);
      display.display();
      char key = keypad.getKey();
      if (key) {
        beep(30);
        if (key >= '0' && key <= '9' && idx < 2) {
          buf[idx++] = key;
        } else if (key == '#' && idx == 2) {
          value = atoi(buf);
          if (value >= min_val && value <= max_val) return true;
          else {
            // invaild
            idx = 0;
            buf[0] = buf[1] = 0;
            display.clearDisplay();
            display.setCursor(0,0);
            display.printf("Invalid! %d-%d", min_val, max_val);
            display.display();
            delay(1000);
          }
        } else if (key == '*') {
          return false; // cancel
        }
      }
      delay(50);
    }
  };

  if (!readTwoDigits("Year (24-99)", year, 24, 99)) {input_cancelled = true;} else {year += 2000;}
  if (!input_cancelled && !readTwoDigits("Month (1-12)", month, 1, 12)) input_cancelled = true;
  if (!input_cancelled && !readTwoDigits("Day (1-31)", day, 1, 31)) input_cancelled = true;
  if (!input_cancelled && !readTwoDigits("Hour (0-23)", hour, 0, 23)) input_cancelled = true;
  if (!input_cancelled && !readTwoDigits("Minute (0-59)", minute, 0, 59)) input_cancelled = true;
  if (!input_cancelled && !readTwoDigits("Second (0-59)", second, 0, 59)) input_cancelled = true;

  if (input_cancelled) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Time set cancelled");
    display.display();
    delay(1000);
    return;
  }

  // Time setting
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.printf("Time set to:\n%04d-%02d-%02d\n%02d:%02d:%02d",
                 year, month, day, hour, minute, second);
  display.display();
  logEvent(5, "RTC time manually set");
  delay(2000);
}

// ==================== Flow ====================
void parseSerialData() {
  static String buffer = "";

  while (SERIAL_FROM_A.available()) {
    char c = SERIAL_FROM_A.read();

    if (c == '\n') {
      buffer.trim();

      Serial.print("RX: ");
      Serial.println(buffer);

      if (buffer.startsWith("FLOW:")) {
        float flow = buffer.substring(5).toFloat();
        last_flow_value = flow;
        flow_sensor_online = true;
        last_flow_receive_time = millis();
      }

      buffer = "";
    } else {
      buffer += c;
    }
  }
}

// ==================== O2 ====================
void readO2Sensor() {
  float voltage_mV = readAD620Voltage(20);
  if (calib.sensitivity != 0) {
    current_data.o2_percent = (voltage_mV - calib.zero_mV) / calib.sensitivity;
  } else {
    current_data.o2_percent = 0;
  }
  current_data.o2_percent = constrain(current_data.o2_percent, 0, 100);
  current_data.timestamp = millis();
  // RTC temp
  if (rtc_available) {
    current_data.temperature = readRTCTemperature();
  } else {
    current_data.temperature = 25.0;
  }
}

float readAD620Voltage(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(O2_ADC_PIN);
    delayMicroseconds(100);
  }
  float avg_adc = sum / (float)samples;
  float adc_voltage = (avg_adc / 4095.0) * 3.3;
  float sensor_voltage = adc_voltage * 1000.0; // mV
  return sensor_voltage;
}

void readFlowSensor() {
  if (flow_sensor_online) {
    current_data.flow_rate = last_flow_value;
  } else {
    current_data.flow_rate = 0.0;
  }
  current_data.flow_rate = constrain(current_data.flow_rate, 0, 20);
}

// ==================== Initialization ====================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\nMedical Oxygen Flow Monitor - Split Version (Serial Flow Input)");

  // Communication with A board
  SERIAL_FROM_A.begin(115200, SERIAL_8N1, RX_FROM_A_PIN, TX_TO_A_PIN);
  Serial.println("Serial2 initialized for flow sensor data");

  initializeSystem();
  initializeSensors();
  if (initializeSDCard()) Serial.println("SD card OK");
  else Serial.println("SD card FAIL");
  initializeRTC();

  if (!loadCalibration()) Serial.println("Using default calibration");

  // initial flow sens status
  flow_sensor_online = false;
  last_flow_receive_time = millis();

  // Restore
  if (!restoreData()) Serial.println("No backup");

  // Start
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);
  display.println("O2 Flow");
  display.setCursor(22, 40);
  display.println("Monitor");
  display.display();
  delay(1000);

  
  readO2Sensor();      // current_data.o2_percent
  readFlowSensor();    // current_data.flow_rate

  // System status
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("System Ready");
  display.println("I2C0: OLED OK");
  display.println("Flow: Serial");
  if (sd_available) display.println("SD: OK");
  else display.println("SD: None");
  if (rtc_available) display.println("RTC: OK");
  else display.println("RTC: None");
  display.display();
  delay(1000);
}

void initializeSystem() {
  Serial.println("Initializing system...");
  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN);
  Wire.setClock(100000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println("OLED NOT FOUND!");
    } else {
      Serial.println("OLED OK (0x3D)");
    }
  } else {
    Serial.println("OLED OK (0x3C)");
  }

  pinMode(ALARM_BUZZER_PIN, OUTPUT);
  digitalWrite(ALARM_BUZZER_PIN, LOW);
  analogReadResolution(12);

  Serial.println("System init done");
}

void initializeSensors() {
  Serial.println("Scanning I2C0 (OLED bus)...");
  int deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("I2C0: Found 0x%02X", address);
      if (address == SCREEN_ADDRESS || address == 0x3D) Serial.println(" (OLED)");
      else if (address == 0x68) Serial.println(" (RTC)");
      else Serial.println(" (Unknown)");
      deviceCount++;
    }
  }
  if (deviceCount == 0) Serial.println("I2C0: No devices found");
  Serial.println("Sensors init done");
}

bool initializeSDCard() {
  Serial.println("Init SD card...");
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  for (int attempts = 1; attempts <= MAX_RETRY_ATTEMPTS; attempts++) {
    Serial.printf("SD try %d/%d...\n", attempts, MAX_RETRY_ATTEMPTS);
    if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(20)))) {
      sd_available = true;
      vol = sd.vol();
      if (!sd.exists("/DATA")) sd.mkdir("/DATA");
      if (!sd.exists("/LOGS")) sd.mkdir("/LOGS");
      if (!sd.exists("/CALIB")) sd.mkdir("/CALIB");
      Serial.println("SD card OK!");
      return true;
    }
    delay(500);
  }
  sd_available = false;
  return false;
}

void initializeRTC() {
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    rtc_available = false;
    return;
  }
  rtc_available = true;
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    logEvent(6, "RTC lost power, reset to compile time");
  }
  Serial.println("RTC OK");
}

// ==================== Calibration ====================
bool loadCalibration() {
  Serial.println("Load calibration...");
  EEPROM.get(CALIB_ZERO_ADDR, calib.zero_mV);
  EEPROM.get(CALIB_SPAN_ADDR, calib.span_mV);
  EEPROM.get(CALIB_SENS_ADDR, calib.sensitivity);
  EEPROM.get(FLOW_CALIB_ADDR, calib.flow_offset);
  EEPROM.get(SFM3300_CALIB_ADDR, calib.sfm3300_offset);
  if (EEPROM.read(SFM3300_CALIB_ADDR + sizeof(float) + 1) != 0xFF) {
    EEPROM.get(SFM3300_CALIB_ADDR + sizeof(float), calib.sfm3300_scale);
  } else {
    calib.sfm3300_scale = 1.0;
  }
  if (isnan(calib.sensitivity) || calib.sensitivity == 0) {
    calib.zero_mV = 0.0;
    calib.span_mV = 50.0;
    calib.span_o2 = 25; //20.9
    calib.sensitivity = 2.392;
    calib.flow_offset = 0.0;
    calib.flow_scale = 1.0;
    calib.sfm3300_offset = 0.0;
    calib.sfm3300_scale = 1.0;
    calib.calib_time = 0;
    Serial.println("Using defaults");
    return false;
  }
  Serial.println("Calibration loaded");
  return true;
}

bool saveCalibration() {
  Serial.println("Save calibration...");
  EEPROM.put(CALIB_ZERO_ADDR, calib.zero_mV);
  EEPROM.put(CALIB_SPAN_ADDR, calib.span_mV);
  EEPROM.put(CALIB_SENS_ADDR, calib.sensitivity);
  EEPROM.put(FLOW_CALIB_ADDR, calib.flow_offset);
  EEPROM.put(SFM3300_CALIB_ADDR, calib.sfm3300_offset);
  EEPROM.put(SFM3300_CALIB_ADDR + sizeof(float), calib.sfm3300_scale);
  if (!EEPROM.commit()) {
    Serial.println("EEPROM commit FAIL!");
    return false;
  }
  if (sd_available) {
    FsFile calFile = sd.open("/CALIB/calibration.csv", FILE_WRITE);
    if (calFile) {
      DateTime now = getCurrentTime();
      if (calFile.fileSize() == 0) {
        calFile.println("timestamp,zero_mV,span_mV,span_o2,sensitivity,flow_offset,flow_scale,sfm3300_offset,sfm3300_scale");
      }
      calFile.printf("%lu,%.4f,%.4f,%.2f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
        now.unixtime(), calib.zero_mV, calib.span_mV, calib.span_o2,
        calib.sensitivity, calib.flow_offset, calib.flow_scale,
        calib.sfm3300_offset, calib.sfm3300_scale);
      calFile.sync();
      calFile.close();
    }
  }
  calib.calib_time = millis();
  logEvent(1, "Calibration done");
  return true;
}

void performCalibration() {
  Serial.println("Calibration start...");
  bool calib_complete = false;
  int calib_step = 0;
  float zero_sum = 0, span_sum = 0;
  int zero_count = 0, span_count = 0;

  while (!calib_complete) {
    showCalibrationScreen();
    char key = keypad.getKey();
    if (key) {
      switch (key) {
        case '1':
          calib_step = 1;
          zero_sum = 0; zero_count = 0;
          break;
        case '2':
          calib_step = 2;
          span_sum = 0; span_count = 0;
          break;
        case '3':
          // SFM3200 calibration removed
          display.clearDisplay();
          display.setCursor(0,0);
          display.println("Flow cal not needed");
          display.display();
          delay(1000);
          break;
        case 'B':
          if (calib_step == 1) {
            for (int i = 0; i < 10; i++) {
              zero_sum += readAD620Voltage(20);
              zero_count++;
              delay(500);
            }
            calib.zero_mV = zero_sum / zero_count;
          } else if (calib_step == 2) {
            for (int i = 0; i < 10; i++) {
              span_sum += readAD620Voltage(20);
              span_count++;
              delay(500);
            }
            calib.span_mV = span_sum / span_count;
            calib.span_o2 = 25; //20.9
            calib.sensitivity = (calib.span_mV - calib.zero_mV) / calib.span_o2;
          }
          break;
        case 'D':
          if (saveCalibration()) {
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(12, 20); display.println("Calib Saved");
            display.setCursor(40, 40); display.println("OK");
            display.display();
            delay(1000);
          } else {
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(12, 20); display.println("Calib Save");
            display.setCursor(40, 40); display.println("FAIL");
            display.display();
            delay(1000);
          }
          calib_complete = true;
          break;
        case 'C':
          calib_complete = true;
          break;
      }
    }
    delay(100);
  }
}

void performValidation() {
  Serial.println("Validation start...");
  showValidationScreen();
  delay(3000);
}

// ==================== Display ====================
void updateDisplay() {
  //char key = keypad.getKey();
  //if (key == 'A') display_mode = (display_mode + 1) % 3;
  switch (display_mode) {
    case 0: showMainScreen(); break;
    case 1: showDetailsScreen(); break;
    case 2: showLogScreen(); break;
  }
}

void showMainScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("O2:%.1f%%\n", current_data.o2_percent);
  display.printf("Flow:%.1fL/m\n", current_data.flow_rate);
  display.setTextSize(1);
  display.setCursor(0, 50);
  if (alarm_active) {
    char short_msg[12];
    strncpy(short_msg, alarm_message, 11);
    short_msg[11] = '\0';
    display.printf("ALRM:%s", short_msg);
  } else {
    DateTime now = getCurrentTime();
    // Display time
    display.printf("%02d/%02d %02d:%02d", now.month(), now.day(), now.hour(), now.minute());
    if (!flow_sensor_online) display.print(" NoFlow");
  }
  display.display();
}

void showDetailsScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== System Info ===");
  display.printf("O2: %.1f %%\n", current_data.o2_percent);
  display.printf("Flow: %.2f L/m\n", current_data.flow_rate);
  display.printf("Sensor: %.2f mV\n", readAD620Voltage(5));
  display.printf("FlowSrc: %s\n", flow_sensor_online ? "Serial" : "OFF");
  if (rtc_available) {
    float temp = readRTCTemperature();
    display.printf("RTC Temp: %.2f C\n", temp);
    DateTime now = rtc.now();
    display.printf("RTC Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   now.year(), now.month(), now.day(),
                   now.hour(), now.minute(), now.second());
  } else {
    display.println("RTC: N/A");
  }
  display.printf("Recs: %d\n", record_count);
  display.printf("File: %d/%d\n", file_record_count, MAX_RECORD_SIZE);
  display.printf("Up: %lu min\n", millis() / 60000);
  if (sd_available) {
    display.print("SD: OK ");
    if (vol) display.printf("(%s)", vol->fatType() == FAT_TYPE_EXFAT ? "exFAT" : "FAT");
  } else {
    display.print("SD: None");
  }
  display.display();
}

void showLogScreen() {
  static uint16_t log_start = 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== Event Log ===");
  for (int i = 0; i < 5; i++) {
    uint16_t addr = 200 + (((log_start + i) % 10) * 70);
    uint32_t timestamp;
    uint8_t type;
    char desc[64] = {0};
    EEPROM.get(addr, timestamp);
    EEPROM.get(addr + 4, type);
    for (int j = 0; j < 63; j++) {
      desc[j] = EEPROM.read(addr + 8 + j);
      if (desc[j] == '\0') break;
    }
    if (timestamp > 0) {
      display.printf("%d: %s\n", i+1, desc);
    }
  }
  display.setCursor(0, 56);
  display.print("A:Prev B:Next C:Back");
  char key = keypad.getKey();
  if (key == 'B') log_start = (log_start + 5) % 10;
  else if (key == 'A') log_start = (log_start - 5 + 10) % 10;
  display.display();
}

void showCalibrationScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== Calibration ===");
  display.println("1: O2 Zero Cal");
  display.println("2: O2 Span Cal");
  display.println("3: Flow Cal (N/A)");
  display.println("B: Measure");
  display.println("D: Save");
  display.println("C: Cancel");
  display.display();
}

void showValidationScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== Validation ===");
  display.println("Checking sensors");
  display.println("Data logging");
  display.println("Alarm functions");
  display.display();
}

void showMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== Main Menu ===");
  display.println("A: Calibrate");
  display.println("B: Validate");
  display.println("C: Event Log");
  display.println("D: Settings");
  display.println("1: Manual Log");
  display.println("4: Reinit Flow? (N/A)");
  display.println("0: Mute Alarm");
  display.println("*: Back");
  display.display();
  bool in_menu = true;
  while (in_menu) {
    char key = keypad.getKey();
    if (key) {
      beep(50);
      processMenu(key);
      in_menu = false;
    }
    delay(100);
  }
}

void showEventLog() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== Event Log ===");
  display.println("Loading from SD...");
  if (sd_available) {
    FsFile logFile = sd.open("/LOGS/events.csv", FILE_READ);
    if (logFile) {
      int line_count = 0;
      while (logFile.available() && line_count < 6) {
        String line = logFile.readStringUntil('\n');
        if (line.length() > 20) line = line.substring(0, 20) + "...";
        display.println(line);
        line_count++;
      }
      logFile.close();
    } else {
      display.println("No events");
    }
  } else {
    display.println("SD not available");
  }
  display.display();
  delay(3000);
}

void showSettingsMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== Settings ===");
  display.println("1: System Info");
  display.println("2: Set RTC Time");
  display.println("C: Back");
  display.display();
  bool in_menu = true;
  while (in_menu) {
    char key = keypad.getKey();
    if (key) {
      beep(50);
      if (key == 'C') in_menu = false;
      else if (key == '1') {
        display.clearDisplay();
        display.setCursor(0,0);
        display.printf("Build: %s\n", __DATE__);
        display.printf("SD: %s\n", sd_available ? "Yes" : "No");
        display.printf("RTC: %s\n", rtc_available ? "Yes" : "No");
        display.printf("Flow: %s\n", flow_sensor_online ? "Online" : "Offline");
        display.printf("EEPROM: %d/%d\n", EEPROM.length(), EEPROM_SIZE);
        display.println("Serial Flow Input");
        if (rtc_available) {
          float temp = readRTCTemperature();
          display.printf("RTC Temp: %.2f C", temp);
        }
        display.display();
        delay(3000);
      }
      else if (key == '2') {
        setRTCtime();
      }
    }
    delay(100);
  }
}

// ==================== Keypad ====================
void handleKeypad() {
  char key = keypad.getKey();
  if (key) {
    last_keypress = millis();
    beep(50);

    if (key == 'A') {
    display_mode = (display_mode + 1) % 3; 
    return;

    }

    processMenu(key);
  }
}

void processMenu(char key) {
  switch (key) {
    case '*': showMainMenu(); break;
    case 'A': performCalibration(); break;
    case 'B': performValidation(); break;
    case 'C': showEventLog(); break;
    case 'D': showSettingsMenu(); break;
    case '0':
      alarm_active = false;
      noTone(ALARM_BUZZER_PIN);
      break;
    case '1':
      if (saveDataToSD()) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(20,20); display.println("Logged");
        display.display();
      } else {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(20,20); display.println("Log FAIL");
        display.display();
      }
      delay(1000);
      break;
    case '4':
      // Flow sensor reinit（unvailable）
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("Flow reinit:");
      display.println("Wait for A board");
      display.display();
      flow_sensor_online = false;
      last_flow_receive_time = millis();
      delay(2000);
      break;
  }
}

// ==================== Data Logging ====================
bool saveDataToSD() {
  if (!sd_available) return false;
  DateTime now = getCurrentTime();
  char filename[30];
  sprintf(filename, "/DATA/data_%04d%02d%02d.csv", now.year(), now.month(), now.day());
  checkFileRotation();
  FsFile dataFile = sd.open(filename, FILE_WRITE);
  if (!dataFile) return false;
  if (dataFile.fileSize() == 0) {
    dataFile.println("timestamp,o2_percent,flow_rate,temp,pressure,status,flow_online");
  }
  dataFile.printf("%lu,%.2f,%.3f,%.2f,%.2f,%d,%d\n",
    now.unixtime(), current_data.o2_percent, current_data.flow_rate,
    current_data.temperature, current_data.pressure, current_data.status,
    flow_sensor_online ? 1 : 0);
  dataFile.sync();
  dataFile.close();
  record_count++;
  file_record_count++;
  return true;
}

void checkFileRotation() {
  if (file_record_count >= MAX_RECORD_SIZE) {
    file_record_count = 0;
    Serial.println("File limit reached");
  }
}

bool logEvent(uint8_t type, const char* desc) {
  Serial.printf("Event: %s\n", desc);
  DateTime now = getCurrentTime();
  if (sd_available) {
    FsFile logFile = sd.open("/LOGS/events.csv", FILE_WRITE);
    if (logFile) {
      if (logFile.fileSize() == 0) logFile.println("timestamp,event_type,description");
      logFile.printf("%lu,%d,\"%s\"\n", now.unixtime(), type, desc);
      logFile.sync();
      logFile.close();
    }
  }
  static uint8_t event_index = 0;
  uint16_t addr = 200 + (event_index * 70);
  EEPROM.put(addr, now.unixtime());
  EEPROM.put(addr + 4, type);
  int len = strlen(desc);
  int write_len = (len < 63) ? len : 63;
  for (int i = 0; i < write_len; i++) EEPROM.write(addr + 8 + i, desc[i]);
  EEPROM.write(addr + 8 + write_len, '\0');
  EEPROM.commit();
  event_index = (event_index + 1) % 10;
  return true;
}

// ==================== Alarm ====================
void checkAlarms() {
  bool new_alarm = false;
  strcpy(alarm_message, "");
  if (current_data.o2_percent < O2_LOW_ALARM) {
    strcat(alarm_message, "O2 LOW ");
    new_alarm = true;
  } else if (current_data.o2_percent > O2_HIGH_ALARM) {
    strcat(alarm_message, "O2 HIGH ");
    new_alarm = true;
  }
  if (current_data.flow_rate < FLOW_LOW_ALARM && current_data.flow_rate > 0.01) {
    strcat(alarm_message, "FLOW LOW ");
    new_alarm = true;
  } else if (current_data.flow_rate > FLOW_HIGH_ALARM) {
    strcat(alarm_message, "FLOW HIGH ");
    new_alarm = true;
  }
  float sensor_voltage = readAD620Voltage(5);
  if (sensor_voltage < 0 || sensor_voltage > 10000) {
    strcat(alarm_message, "SENS ERR ");
    new_alarm = true;
  }
  if (!flow_sensor_online) {
    strcat(alarm_message, "FLOW SENS ");
    new_alarm = true;
  }
  if (new_alarm && !alarm_active) {
    alarm_active = true;
    logEvent(2, alarm_message);
    tone(ALARM_BUZZER_PIN, 1000, 500);
  } else if (!new_alarm && alarm_active) {
    alarm_active = false;
    logEvent(2, "Alarm cleared");
  }
}

// ==================== Backup ====================
bool backupData() {
  backup_data = current_data;
  EEPROM.put(LAST_RECORD_ADDR, backup_data);
  if (!EEPROM.commit()) {
    Serial.println("Backup FAIL!");
    return false;
  }
  Serial.println("Backup done");
  return true;
}

bool restoreData() {
  EEPROM.get(LAST_RECORD_ADDR, backup_data);
  if (backup_data.timestamp > 0 && backup_data.o2_percent >= 0 && backup_data.o2_percent <= 100) {
    Serial.println("Backup found");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.printf("Restored:\nO2: %.1f%%\nFlow: %.2f L/m\n", backup_data.o2_percent, backup_data.flow_rate);
    display.display();
    delay(2000);
    logEvent(3, "Restore done");
    return true;
  } else {
    Serial.println("No backup");
    return false;
  }
}

// ==================== LOOP ====================
void loop() {
  uint32_t current_time = millis();

  // Serial data
  parseSerialData();

  // Sampling
  if (current_time - last_sample >= SAMPLE_INTERVAL) {
    readO2Sensor();
    readFlowSensor();
    last_sample = current_time;
    checkAlarms();
  }

  updateDisplay();
  handleKeypad();

  if (current_time - last_log >= LOG_INTERVAL) {
    if (saveDataToSD()) Serial.println("Data logged");
    else Serial.println("Log FAIL");
    last_log = current_time;
  }

  if (current_time - last_backup >= BACKUP_INTERVAL) {
    if (backupData()) Serial.println("Backup OK");
    else Serial.println("Backup FAIL");
    last_backup = current_time;
  }

  delay(10);
}
