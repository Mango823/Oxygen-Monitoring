#include <Wire.h>

// I2C Pins 
#define I2C1_SDA 21
#define I2C1_SCL 22

// SFM3300 
#define SFM3300_ADDR 0x40
#define CMD_START_MEAS 0x1000
#define CMD_RESET      0x2000

bool sfm_ok = false;

// CRC8 
uint8_t crc8(uint8_t *data, int len)
{
  uint8_t crc = 0x00;
  uint8_t poly = 0x31;

  for (int i = 0; i < len; i++) {
    crc ^= data[i];

    for (int j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ poly;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Send Command 
bool sendCommand(uint16_t cmd)
{
  Wire1.beginTransmission(SFM3300_ADDR);
  Wire1.write(cmd >> 8);
  Wire1.write(cmd & 0xFF);

  if (Wire1.endTransmission() == 0)
    return true;

  return false;
}

//  Read Flow 
bool readFlow(float &flow)
{
  Wire1.requestFrom(SFM3300_ADDR, 3);

  if (Wire1.available() < 3)
    return false;

  uint8_t msb = Wire1.read();
  uint8_t lsb = Wire1.read();
  uint8_t crc = Wire1.read();

  uint8_t data[2] = {msb, lsb};

  if (crc8(data, 2) != crc) {
    Serial.println("CRC error");
    return false;
  }

  uint16_t raw = (msb << 8) | lsb;

  // datasheet conversion
  flow = (raw - 32768.0) / 120.0;

  return true;
}

// Setup
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  delay(1000);

  Serial.println("SFM3300 Test");

  Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setClock(100000);

  sendCommand(CMD_RESET);
  delay(50);

  if (sendCommand(CMD_START_MEAS)) {
    Serial.println("Measurement started");
    sfm_ok = true;
  }
  else {
    Serial.println("Sensor not responding");
  }

  delay(100);
}

//Loop 
void loop()
{
  if (!sfm_ok) {
    delay(1000);
    return;
  }

  float flow;

  if (readFlow(flow)) {
    Serial2.print("FLOW: ");
    Serial2.println(flow);
    //Serial.println(" L/min");
  }
  else {
    Serial.println("Read failed");
  }

  delay(1000);
}