#include "IMU.h"

#include <Wire.h>

namespace
{
const uint8_t IMU_ADDR = 0x68;
const uint8_t IMU_STATUS_LED_PIN = LED_BUILTIN;
const uint8_t REG_WHO_AM_I = 0x75;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_PWR_MGMT_2 = 0x6C;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

const float ACCEL_SCALE_MPS2 = 9.80665f / 4096.0f; // +-8g
const float GYRO_SCALE_RADPS = DEG_TO_RAD / 65.5f; // +-500 dps
const int GYRO_CAL_SAMPLES = 500;

bool g_imuReady = false;
uint8_t g_whoAmI = 0x00;

float g_headingRad = 0.0f;
float g_headingDeg = 0.0f;

float g_magX_uT = 0.0f;
float g_magY_uT = 0.0f;
float g_magZ_uT = 0.0f;

float g_gyroX_radps = 0.0f;
float g_gyroY_radps = 0.0f;
float g_gyroZ_radps = 0.0f;

float g_accelX_mps2 = 0.0f;
float g_accelY_mps2 = 0.0f;
float g_accelZ_mps2 = 0.0f;

float g_gyroBiasZ_radps = 0.0f;
unsigned long g_lastUpdateUs = 0;
unsigned long g_lastHeartbeatMs = 0;
bool g_statusLedState = false;

float wrapPi(float angle)
{
  while (angle > PI)
    angle -= 2.0f * PI;
  while (angle < -PI)
    angle += 2.0f * PI;
  return angle;
}

float wrap360(float degrees)
{
  while (degrees >= 360.0f)
    degrees -= 360.0f;
  while (degrees < 0.0f)
    degrees += 360.0f;
  return degrees;
}

void setStatusLed(bool on)
{
  g_statusLedState = on;
  digitalWrite(IMU_STATUS_LED_PIN, on ? HIGH : LOW);
}

void toggleStatusLed()
{
  setStatusLed(!g_statusLedState);
}

void updateReadyHeartbeat()
{
  const unsigned long now = millis();
  const unsigned long heartbeatIntervalMs = 500;

  if (now - g_lastHeartbeatMs >= heartbeatIntervalMs)
  {
    g_lastHeartbeatMs = now;
    toggleStatusLed();
  }
}

void scanI2cBus()
{
  Serial.println(F("I2C scan start"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0)
    {
      Serial.print(F("I2C device found at 0x"));
      if (addr < 16)
      {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0)
  {
    Serial.println(F("No I2C devices found"));
  }
}

bool writeI2cRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readI2cRegister(uint8_t reg, uint8_t *value)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  uint8_t count = Wire.requestFrom((int)IMU_ADDR, 1);
  if (count != 1 || !Wire.available())
  {
    return false;
  }

  *value = Wire.read();
  return true;
}

bool readImuBlock(uint8_t startReg, uint8_t *buffer, uint8_t len)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  uint8_t count = Wire.requestFrom((int)IMU_ADDR, (int)len);
  if (count != len)
  {
    return false;
  }

  for (uint8_t i = 0; i < len; i++)
  {
    if (!Wire.available())
    {
      return false;
    }
    buffer[i] = Wire.read();
  }

  return true;
}

bool isSupportedWhoAmI(uint8_t whoAmI)
{
  switch (whoAmI)
  {
  case 0x68:
  case 0x69:
  case 0x70:
  case 0x71:
  case 0x73:
    return true;
  default:
    return false;
  }
}

void printWhoAmI()
{
  Serial.print(F("WHO_AM_I at 0x68 = 0x"));
  if (g_whoAmI < 16)
  {
    Serial.print('0');
  }
  Serial.println(g_whoAmI, HEX);
}

bool configureImuRegisters()
{
  if (!writeI2cRegister(REG_PWR_MGMT_1, 0x80))
    return false;
  delay(100);
  if (!writeI2cRegister(REG_PWR_MGMT_1, 0x01))
    return false;
  if (!writeI2cRegister(REG_PWR_MGMT_2, 0x00))
    return false;
  if (!writeI2cRegister(REG_CONFIG, 0x04))
    return false;
  if (!writeI2cRegister(REG_GYRO_CONFIG, 0x08))
    return false;
  if (!writeI2cRegister(REG_ACCEL_CONFIG, 0x10))
    return false;
  if (!writeI2cRegister(REG_ACCEL_CONFIG2, 0x04))
    return false;
  delay(10);
  return true;
}

bool readImuSample()
{
  uint8_t data[14];
  if (!readImuBlock(REG_ACCEL_XOUT_H, data, sizeof(data)))
  {
    return false;
  }

  int16_t ax = (int16_t)((data[0] << 8) | data[1]);
  int16_t ay = (int16_t)((data[2] << 8) | data[3]);
  int16_t az = (int16_t)((data[4] << 8) | data[5]);
  int16_t gx = (int16_t)((data[8] << 8) | data[9]);
  int16_t gy = (int16_t)((data[10] << 8) | data[11]);
  int16_t gz = (int16_t)((data[12] << 8) | data[13]);

  g_accelX_mps2 = (float)ax * ACCEL_SCALE_MPS2;
  g_accelY_mps2 = (float)ay * ACCEL_SCALE_MPS2;
  g_accelZ_mps2 = (float)az * ACCEL_SCALE_MPS2;

  g_gyroX_radps = (float)gx * GYRO_SCALE_RADPS;
  g_gyroY_radps = (float)gy * GYRO_SCALE_RADPS;
  g_gyroZ_radps = (float)gz * GYRO_SCALE_RADPS;

  // This chip path is gyro-only for heading right now.
  g_magX_uT = 0.0f;
  g_magY_uT = 0.0f;
  g_magZ_uT = 0.0f;
  return true;
}
} // namespace

bool setupImu()
{
  pinMode(IMU_STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);

  Wire.begin();
  Wire.setClock(400000);

  if (!readI2cRegister(REG_WHO_AM_I, &g_whoAmI))
  {
    Serial.println(F("IMU WHO_AM_I read failed"));
    scanI2cBus();
    g_imuReady = false;
    return false;
  }

  printWhoAmI();
  if (!isSupportedWhoAmI(g_whoAmI))
  {
    Serial.println(F("Unsupported IMU type"));
    g_imuReady = false;
    return false;
  }

  if (!configureImuRegisters())
  {
    Serial.println(F("IMU register configuration failed"));
    g_imuReady = false;
    return false;
  }

  g_gyroBiasZ_radps = 0.0f;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
  {
    if ((i % 50) == 0)
    {
      toggleStatusLed();
    }

    if (!readImuSample())
    {
      Serial.println(F("IMU gyro calibration failed"));
      setStatusLed(false);
      g_imuReady = false;
      return false;
    }
    g_gyroBiasZ_radps += g_gyroZ_radps;
    delay(2);
  }
  g_gyroBiasZ_radps /= (float)GYRO_CAL_SAMPLES;

  g_headingRad = 0.0f;
  g_headingDeg = 0.0f;
  g_lastUpdateUs = micros();
  g_lastHeartbeatMs = millis();
  g_imuReady = true;

  Serial.println(F("IMU ready"));
  Serial.print(F("IMU gyro bias z rad/s: "));
  Serial.println(g_gyroBiasZ_radps, 6);
  setStatusLed(true);
  return true;
}

void updateImu()
{
  if (!g_imuReady)
  {
    return;
  }

  updateReadyHeartbeat();

  if (!readImuSample())
  {
    return;
  }

  unsigned long nowUs = micros();
  float dt = (nowUs - g_lastUpdateUs) / 1000000.0f;
  g_lastUpdateUs = nowUs;

  if (dt > 0.0f && dt < 0.2f)
  {
    float yawRate = g_gyroZ_radps - g_gyroBiasZ_radps;
    if (fabs(yawRate) < 0.01f)
    {
      yawRate = 0.0f;
    }

    g_gyroZ_radps = yawRate;
    g_headingRad = wrapPi(g_headingRad + yawRate * dt);
    g_headingDeg = wrap360(g_headingRad * 180.0f / PI);
  }
}

bool isImuReady()
{
  return g_imuReady;
}

float getImuHeadingRad()
{
  return g_headingRad;
}

float getImuHeadingDeg()
{
  return g_headingDeg;
}

float getImuMagX_uT()
{
  return g_magX_uT;
}

float getImuMagY_uT()
{
  return g_magY_uT;
}

float getImuMagZ_uT()
{
  return g_magZ_uT;
}

float getImuGyroX_radps()
{
  return g_gyroX_radps;
}

float getImuGyroY_radps()
{
  return g_gyroY_radps;
}

float getImuGyroZ_radps()
{
  return g_gyroZ_radps;
}

float getImuAccelX_mps2()
{
  return g_accelX_mps2;
}

float getImuAccelY_mps2()
{
  return g_accelY_mps2;
}

float getImuAccelZ_mps2()
{
  return g_accelZ_mps2;
}
