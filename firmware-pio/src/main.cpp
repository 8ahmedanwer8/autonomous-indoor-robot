#include <Arduino.h>
#include "DeviceDriverSet_xxx0.h"
// #include <pinch
#include "odometry.h"

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
  setCommandSigns(+1, +1);
  Motor.DeviceDriverSet_Motor_control(direction_just, spd,
                                      direction_just, spd, control_enable);
}
static void motorsBackward(uint8_t spd)
{
  setCommandSigns(-1, -1);
  Motor.DeviceDriverSet_Motor_control(direction_back, spd,
                                      direction_back, spd, control_enable);
}
static void motorsLeft(uint8_t spd)
{
  setCommandSigns(-1, +1);
  Motor.DeviceDriverSet_Motor_control(direction_back, spd, // A
                                      direction_just, spd, // B
                                      control_enable);
}
static void motorsRight(uint8_t spd)
{
  setCommandSigns(+1, -1);
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

        setCommandSigns((L >= 0) ? +1 : -1, (R >= 0) ? +1 : -1);
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
  setupOdometry();
  Motor.DeviceDriverSet_Motor_Init();
  AppIRrecv.DeviceDriverSet_IRrecv_Init();

  g_mode = MODE_STANDBY;
  motorsStop();
  lastCmdMs = millis();
}

void loop()
{
  updateOdometry();
  uint8_t btn;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&btn))
  {
    handleButton(btn);
    lastCmdMs = millis();
  }

  handleSerialJetson();
}
