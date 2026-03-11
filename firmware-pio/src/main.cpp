#include <Arduino.h>
#include "DeviceDriverSet_xxx0.h"
// #include <pinch
#include "PinChangeInterrupt.h"

const byte ENC_BL_PIN = 12; // left encoder signal pin
const byte ENC_BR_PIN = 11; // right encoder signal pin (pick a free pin)
const byte ENC_FR_PIN = 10;
const byte ENC_FL_PIN = 4;

volatile unsigned long encPulsesBL = 0;
volatile unsigned long encPulsesBR = 0;
volatile unsigned long encPulsesFR = 0;
volatile unsigned long encPulsesFL = 0;

volatile int8_t cmdSignL = 1;
volatile int8_t cmdSignR = 1;

void encoderISR_BL() { encPulsesBL++; }
void encoderISR_BR() { encPulsesBR++; }
void encoderISR_FR() { encPulsesFR++; }
void encoderISR_FL() { encPulsesFL++; }

// ---- ODOMETRY CONFIG ----
const float WHEEL_CIRC_M = 0.215f;  // <-- set this (meters). Example: 204mm = 0.204m
const uint16_t PULSES_PER_REV = 8;  // <-- set this after 1-rev calibration
const bool LOG_RAW_DIGITAL = false; // set true if you still want >enc raw 0/1
const float TRACK_WIDTH_M = 0.13f;  // distance between center of right tire and center of left tire

// ---- ODOMETRY STATE ----
float totalDistL_M = 0.0f;
float totalDistR_M = 0.0f;
float x = 0, y = 0, theta = 0;

void logOdometry()
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

  // snapshot counts safely
  noInterrupts();
  unsigned long cBL = encPulsesBL;
  unsigned long cBR = encPulsesBR;
  unsigned long cFL = encPulsesFL;
  unsigned long cFR = encPulsesFR;
  int8_t sL = cmdSignL; // for now you can keep using BL as “left sign”
  int8_t sR = cmdSignR; // and BR as “right sign”
  interrupts();

  unsigned long dPBL = cBL - lastBL;
  unsigned long dPBR = cBR - lastBR;
  unsigned long dPFL = cFL - lastFL;
  unsigned long dPFR = cFR - lastFR;

  float dPL = 0.5f * ((float)dPBL + (float)dPFL); // left side pulses this interval
  float dPR = 0.5f * ((float)dPBR + (float)dPFR); // right side pulses this interval

  lastBL = cBL;
  lastBR = cBR;
  lastFL = cFL;
  lastFR = cFR;

  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  const float distPerPulse = WHEEL_CIRC_M / (float)PULSES_PER_REV;

  float dL = sL * (dPL * distPerPulse);
  float dR = sR * (dPR * distPerPulse);

  float ds = 0.5f * (dR + dL);
  float dtheta = (dR - dL) / TRACK_WIDTH_M;

  float theta_mid = theta + 0.5f * dtheta;
  x += ds * cos(theta_mid);
  y += ds * sin(theta_mid);
  theta += dtheta;

  if (theta > PI)
    theta -= 2 * PI;
  else if (theta < -PI)
    theta += 2 * PI;

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
  Serial.println(x);
  Serial.print(">Y:");
  Serial.println(y);
  Serial.print(">TH:");
  Serial.println(theta);

  totalDistL_M += dL;
  totalDistR_M += dR;

  float vL = (dt > 0) ? (dL / dt) : 0;
  float vR = (dt > 0) ? (dR / dt) : 0;

  // simple “overall” numbers (still no trig)
  float vAvg = 0.5f * (vL + vR);
  float distAvg = 0.5f * (totalDistL_M + totalDistR_M);

  // Teleplot-friendly outputs
  // Serial.print(">dPL:");
  // Serial.println((long)dPL);
  // Serial.print(">dPR:");
  // Serial.println((long)dPR);

  // Serial.print(">vL:");
  // Serial.println(vL);
  // Serial.print(">vR:");
  // Serial.println(vR);
  // Serial.print(">v:");
  // Serial.println(vAvg);

  // Serial.print(">xL:");
  // Serial.println(totalDistL_M);
  // Serial.print(">xR:");
  // Serial.println(totalDistR_M);
  // Serial.print(">x:");
  // Serial.println(distAvg);

  // Serial.print(">pL:");
  // Serial.println(cBL);
  // Serial.print(">pR:");
  // Serial.println(cBR);
}

void setupEncoder()
{
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_FL_PIN, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(ENC_BL_PIN), encoderISR_BL, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_BR_PIN), encoderISR_BR, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_FR_PIN), encoderISR_FR, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_FL_PIN), encoderISR_FL, CHANGE);

  // If counts look doubled/noisy later, switch to RISING:
  // attachPCINT(digitalPinToPCINT(ENC_BL_PIN), encoderISR_L, RISING);
  // attachPCINT(digitalPinToPCINT(ENC_BR_PIN), encoderISR_R, RISING);
}

#define BAUD 115200
#define CMD_TIMEOUT_MS 800

enum Mode : uint8_t
{
  MODE_STANDBY,
  MODE_DRIVE,
  MODE_DRIVE_NUDGE
};

static Mode g_mode = MODE_STANDBY;
static uint32_t lastCmdMs = 0;

uint8_t g_speed = 130;

DeviceDriverSet_Motor Motor;
DeviceDriverSet_IRrecv AppIRrecv;

// ---- Motor helpers ----
static void motorsStop()
{
  Motor.DeviceDriverSet_Motor_control(direction_void, 0,
                                      direction_void, 0, control_enable);
}

static void motorsForward(uint8_t spd)
{
  cmdSignL = +1;
  cmdSignR = +1;
  Motor.DeviceDriverSet_Motor_control(direction_just, spd,
                                      direction_just, spd, control_enable);
}
static void motorsBackward(uint8_t spd)
{
  cmdSignL = -1;
  cmdSignR = -1;
  Motor.DeviceDriverSet_Motor_control(direction_back, spd,
                                      direction_back, spd, control_enable);
}
static void motorsLeft(uint8_t spd)
{
  cmdSignL = -1;
  cmdSignR = +1;
  Motor.DeviceDriverSet_Motor_control(direction_back, spd, // A
                                      direction_just, spd, // B
                                      control_enable);
}
static void motorsRight(uint8_t spd)
{
  cmdSignL = +1;
  cmdSignR = -1;
  Motor.DeviceDriverSet_Motor_control(direction_just, spd, // A
                                      direction_back, spd, // B
                                      control_enable);
}

// Map Elegoo button IDs → actions
static void handleButton(uint8_t b)
{
  switch (b)
  {
  case 1:
    g_mode = MODE_DRIVE;
    motorsForward(g_speed);
    break;
  case 2:
    g_mode = MODE_DRIVE;
    motorsBackward(g_speed);
    break;
  case 3:
    g_mode = MODE_DRIVE;
    motorsLeft(g_speed);
    break;
  case 4:
    g_mode = MODE_DRIVE;
    motorsRight(g_speed);
    break;
  case 5:
    g_mode = MODE_STANDBY;
    motorsStop();
    break;
  case 6:
    if (g_speed < 250)
      g_speed += 5;
    break;
  case 7:
    g_speed = 160;
    break;
  case 8:
    if (g_speed > 50)
      g_speed -= 5;
    break;
  case 9:
    g_mode = (g_mode == MODE_DRIVE) ? MODE_DRIVE_NUDGE : MODE_DRIVE;
    break;
  default:
    break;
  }
}

void handleSerialJetson()
{
  // Read and handle one line if available
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.equalsIgnoreCase("STOP"))
    {
      g_mode = MODE_STANDBY;
      motorsStop();
      lastCmdMs = millis();
      return;
    }

    if (line.startsWith("VEL,"))
    {
      g_mode = MODE_DRIVE;

      int c1 = line.indexOf(',', 4);
      if (c1 > 0)
      {
        int L = line.substring(4, c1).toInt();
        int R = line.substring(c1 + 1).toInt();

        L = constrain(L, -255, 255); // fix L/R vs A/B
        R = constrain(R, -255, 255);

        cmdSignL = (L >= 0) ? +1 : -1;
        cmdSignR = (R >= 0) ? +1 : -1;

        // A = Right, B = Left (per your driver comments)
        boolean dirA = (R >= 0) ? direction_just : direction_back;
        boolean dirB = (L >= 0) ? direction_just : direction_back;

        uint8_t spdA = (uint8_t)min(abs(R), 255);
        uint8_t spdB = (uint8_t)min(abs(L), 255);

        Motor.DeviceDriverSet_Motor_control(dirA, spdA, dirB, spdB, control_enable);
        lastCmdMs = millis();
      }
      return;
    }

    if (line.startsWith("STEP,"))
    {
      int p1 = line.indexOf(',', 5);
      int p2 = (p1 > 0) ? line.indexOf(',', p1 + 1) : -1;
      if (p1 > 0 && p2 > 0)
      {
        char dir = line.substring(5, p1)[0];
        int spd = constrain(line.substring(p1 + 1, p2).toInt(), 0, 255);
        int ms = constrain(line.substring(p2 + 1).toInt(), 20, 1000);

        g_mode = MODE_DRIVE;

        if (dir == 'F')
          motorsForward(spd);
        else if (dir == 'B')
          motorsBackward(spd);
        else if (dir == 'L')
          motorsLeft(spd);
        else if (dir == 'R')
          motorsRight(spd);
        else
          motorsStop();

        delay(ms);
        motorsStop();
        lastCmdMs = millis();
      }
      return;
    }
  }

  // Timeout always runs (even if no serial available)
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS)
  {
    g_mode = MODE_STANDBY;
    motorsStop();
    // DO NOT update lastCmdMs here
  }
}

void setup()
{
  Serial.begin(BAUD);
  setupEncoder();
  Motor.DeviceDriverSet_Motor_Init();
  AppIRrecv.DeviceDriverSet_IRrecv_Init();

  g_mode = MODE_STANDBY;
  motorsStop();
  lastCmdMs = millis();
}

void loop()
{
  logOdometry();
  uint8_t btn;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&btn))
  {
    handleButton(btn);
    lastCmdMs = millis();
  }

  handleSerialJetson();
}