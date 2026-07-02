#include "odometry.h"
#include "IMU.h"
#include "PinChangeInterrupt.h"
#include "telemetry_config.h"
#include <Arduino.h>

const byte ENC_BL_PIN = 12;
const byte ENC_BR_PIN = 11;
const byte ENC_FR_PIN = 10;
const byte ENC_FL_PIN = 4;

volatile unsigned long encPulsesBL = 0;
volatile unsigned long encPulsesBR = 0;
volatile unsigned long encPulsesFR = 0;
volatile unsigned long encPulsesFL = 0;

volatile int8_t cmdSignL = 1;
volatile int8_t cmdSignR = 1;

const float WHEEL_CIRC_M = 0.215f;  // <-- set this (meters). Example: 204mm = 0.204m
const uint16_t PULSES_PER_REV = 8;  // <-- set this after 1-rev calibration
static float totalDistL_M = 0.0f;
static float totalDistR_M = 0.0f;
static float x = 0, y = 0, theta = 0;

static float wrapPi(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

static void sendTeleplotOdomDebug(unsigned long dPBL, unsigned long dPBR,
                                  unsigned long dPFL, unsigned long dPFR,
                                  float dPL, float dPR,
                                  float xNow, float yNow)
{
#if TELEMETRY_MODE == TELEMETRY_MODE_TELEPLOT_DEBUG
    Serial.print(">dPBL:");
    Serial.println((long)dPBL);
    Serial.print(">dPFL:");
    Serial.println((long)dPFL);
    Serial.print(">dPBR:");
    Serial.println((long)dPBR);
    Serial.print(">dPFR:");
    Serial.println((long)dPFR);
    Serial.print(">dPL:");
    Serial.println(dPL);
    Serial.print(">dPR:");
    Serial.println(dPR);
    Serial.print(">X:");
    Serial.println(xNow, 6);
    Serial.print(">Y:");
    Serial.println(yNow, 6);
#endif
}

void encoderISR_BL() { encPulsesBL++; }
void encoderISR_BR() { encPulsesBR++; }
void encoderISR_FR() { encPulsesFR++; }
void encoderISR_FL() { encPulsesFL++; }

void setupOdometry()
{
    pinMode(ENC_BL_PIN, INPUT_PULLUP);
    pinMode(ENC_BR_PIN, INPUT_PULLUP);
    pinMode(ENC_FR_PIN, INPUT_PULLUP);
    pinMode(ENC_FL_PIN, INPUT_PULLUP);

    attachPCINT(digitalPinToPCINT(ENC_BL_PIN), encoderISR_BL, CHANGE);
    attachPCINT(digitalPinToPCINT(ENC_BR_PIN), encoderISR_BR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENC_FR_PIN), encoderISR_FR, CHANGE);
    attachPCINT(digitalPinToPCINT(ENC_FL_PIN), encoderISR_FL, CHANGE);
}

void setCommandSigns(int8_t leftSign, int8_t rightSign)
{
    noInterrupts();
    cmdSignL = leftSign;
    cmdSignR = rightSign;
    interrupts();
}

void updateOdometry()
{
    static unsigned long lastMs = 0;
    static unsigned long lastBL = 0, lastBR = 0, lastFL = 0, lastFR = 0;

    unsigned long now = millis();
    if (lastMs == 0)
    {
        lastMs = now;
        return;
    }

    const unsigned long intervalMs = 100; // 10 Hz

    if (now - lastMs < intervalMs)
        return;

    noInterrupts();
    unsigned long cBL = encPulsesBL;
    unsigned long cBR = encPulsesBR;
    unsigned long cFL = encPulsesFL;
    unsigned long cFR = encPulsesFR;
    int8_t sL = cmdSignL;
    int8_t sR = cmdSignR;
    interrupts();

    unsigned long dPBL = cBL - lastBL;
    unsigned long dPBR = cBR - lastBR;
    unsigned long dPFL = cFL - lastFL;
    unsigned long dPFR = cFR - lastFR;

    float dPL = 0.5f * ((float)dPBL + (float)dPFL);
    float dPR = 0.5f * ((float)dPBR + (float)dPFR);

    lastBL = cBL;
    lastBR = cBR;
    lastFL = cFL;
    lastFR = cFR;

    lastMs = now;

    const float distPerPulse = WHEEL_CIRC_M / (float)PULSES_PER_REV;

    float dL = sL * (dPL * distPerPulse);
    float dR = sR * (dPR * distPerPulse);

    float ds = 0.5f * (dR + dL);
    float prevTheta = theta;
    float imuTheta = isImuReady() ? getImuHeadingRad() : theta;
    float dtheta = wrapPi(imuTheta - prevTheta);
    float theta_mid = wrapPi(prevTheta + 0.5f * dtheta);
    x += ds * cos(theta_mid);
    y += ds * sin(theta_mid);
    theta = imuTheta;
    totalDistL_M += dL;
    totalDistR_M += dR;
    sendTeleplotOdomDebug(dPBL, dPBR, dPFL, dPFR, dPL, dPR, x, y);
}

float getOdomX() { return x; }
float getOdomY() { return y; }
float getOdomTheta() { return theta; }
float getLeftDistanceM() { return totalDistL_M; }
float getRightDistanceM() { return totalDistR_M; }

unsigned long getLeftPulses()
{
    noInterrupts();
    unsigned long leftPulses = (unsigned long)(0.5f * ((float)encPulsesBL + (float)encPulsesFL));
    interrupts();
    return leftPulses;
}

unsigned long getRightPulses()
{
    noInterrupts();
    unsigned long rightPulses = (unsigned long)(0.5f * ((float)encPulsesBR + (float)encPulsesFR));
    interrupts();
    return rightPulses;
}
