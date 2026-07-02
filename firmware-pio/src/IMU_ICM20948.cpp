#include "IMU_ICM20948.h"
#include "telemetry_config.h"

#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

namespace
{
const uint8_t IMU_STATUS_LED_PIN = LED_BUILTIN;
const uint8_t IMU_ADDR_DEFAULT = 0x69;
const uint8_t IMU_ADDR_ALT = 0x68;
const int GYRO_CAL_SAMPLES = 500;

Adafruit_ICM20948 g_icm;
bool g_imuReady = false;
uint8_t g_i2cAddr = 0x00;

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
unsigned long g_lastDebugLogMs = 0;
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

bool probeI2cAddress(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool beginImuAt(uint8_t addr)
{
  Serial.print(F("Probing ICM20948 at 0x"));
  if (addr < 16)
  {
    Serial.print('0');
  }
  Serial.println(addr, HEX);

  if (!probeI2cAddress(addr))
  {
    Serial.println(F("No I2C ACK at that address"));
    return false;
  }

  if (!g_icm.begin_I2C(addr, &Wire))
  {
    Serial.println(F("I2C ACK received, but Adafruit init failed"));
    return false;
  }

  g_i2cAddr = addr;
  g_icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  g_icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  g_icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  return true;
}

bool readImuSample()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;

  if (!g_icm.getEvent(&accel, &gyro, &temp, &mag))
  {
    return false;
  }

  g_accelX_mps2 = accel.acceleration.x;
  g_accelY_mps2 = accel.acceleration.y;
  g_accelZ_mps2 = accel.acceleration.z;

  g_gyroX_radps = gyro.gyro.x;
  g_gyroY_radps = gyro.gyro.y;
  g_gyroZ_radps = gyro.gyro.z;

  g_magX_uT = mag.magnetic.x;
  g_magY_uT = mag.magnetic.y;
  g_magZ_uT = mag.magnetic.z;
  return true;
}
} // namespace

bool setupImuIcm20948()
{
  pinMode(IMU_STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);

  Wire.begin();
  delay(10);
  Wire.setClock(100000);

  if (!beginImuAt(IMU_ADDR_DEFAULT) && !beginImuAt(IMU_ADDR_ALT))
  {
    Serial.println(F("ICM20948 init failed at 0x69 and 0x68"));
    scanI2cBus();
    g_imuReady = false;
    return false;
  }

  Serial.print(F("ICM20948 ready on I2C address 0x"));
  if (g_i2cAddr < 16)
  {
    Serial.print('0');
  }
  Serial.println(g_i2cAddr, HEX);

  g_gyroBiasZ_radps = 0.0f;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
  {
    if ((i % 50) == 0)
    {
      toggleStatusLed();
    }

    if (!readImuSample())
    {
      Serial.println(F("ICM20948 gyro calibration failed"));
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
  g_lastDebugLogMs = 0;
  g_imuReady = true;

  Serial.print(F("ICM20948 gyro bias z rad/s: "));
  Serial.println(g_gyroBiasZ_radps, 6);
  setStatusLed(true);
  return true;
}

void updateImuIcm20948()
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

#if TELEMETRY_MODE == TELEMETRY_MODE_TELEPLOT_DEBUG
  const unsigned long nowMs = millis();
  if (nowMs - g_lastDebugLogMs >= 100)
  {
    g_lastDebugLogMs = nowMs;
    Serial.print(">IMU_GZ:");
    Serial.println(g_gyroZ_radps, 6);
    Serial.print(">IMU_MX:");
    Serial.println(g_magX_uT, 6);
    Serial.print(">IMU_MY:");
    Serial.println(g_magY_uT, 6);
    Serial.print(">IMU_MZ:");
    Serial.println(g_magZ_uT, 6);
    Serial.print(">IMU_TH:");
    Serial.println(g_headingRad, 6);
  }
#endif
}

bool isImuIcm20948Ready()
{
  return g_imuReady;
}

float getImuIcm20948HeadingRad()
{
  return g_headingRad;
}

float getImuIcm20948HeadingDeg()
{
  return g_headingDeg;
}

float getImuIcm20948MagX_uT()
{
  return g_magX_uT;
}

float getImuIcm20948MagY_uT()
{
  return g_magY_uT;
}

float getImuIcm20948MagZ_uT()
{
  return g_magZ_uT;
}

float getImuIcm20948GyroX_radps()
{
  return g_gyroX_radps;
}

float getImuIcm20948GyroY_radps()
{
  return g_gyroY_radps;
}

float getImuIcm20948GyroZ_radps()
{
  return g_gyroZ_radps;
}

float getImuIcm20948AccelX_mps2()
{
  return g_accelX_mps2;
}

float getImuIcm20948AccelY_mps2()
{
  return g_accelY_mps2;
}

float getImuIcm20948AccelZ_mps2()
{
  return g_accelZ_mps2;
}
