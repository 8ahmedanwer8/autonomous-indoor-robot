#include <Arduino.h>
#include "DeviceDriverSet_xxx0.h"
#include "IMU.h"
#include "odometry.h"
#include "telemetry_config.h"

#define BAUD 115200
#define CMD_TIMEOUT_MS 400
#define ODOM_TX_INTERVAL_MS 100

enum Mode : uint8_t
{
  MODE_STANDBY,
  MODE_DRIVE,
  MODE_DRIVE_NUDGE
};

static Mode g_mode = MODE_STANDBY;
static uint32_t lastCmdMs = 0;
static uint8_t lastIrBtn = 0;
static uint32_t lastIrBtnMs = 0;

uint8_t g_speed = 90; // reduced from 130 for testing

DeviceDriverSet_Motor Motor;
DeviceDriverSet_IRrecv AppIRrecv;

struct PinTestState
{
  bool active;
  uint8_t stby;
  uint8_t a1;
  uint8_t a2;
  uint8_t pwmA;
  uint8_t b1;
  uint8_t b2;
  uint8_t pwmB;
};

static PinTestState g_pinTest = {false, 0, 0, 0, 0, 0, 0, 0};

static bool isMotionButton(uint8_t b)
{
  return b >= 1 && b <= 5;
}

static void sendOdomPacket()
{
#if TELEMETRY_MODE == TELEMETRY_MODE_ROS_PACKET
  static uint32_t lastTxMs = 0;

  const uint32_t now = millis();
  if (now - lastTxMs < ODOM_TX_INTERVAL_MS)
  {
    return;
  }

  lastTxMs = now;

  const unsigned long leftPulses = getLeftPulses();
  const unsigned long rightPulses = getRightPulses();

  Serial.print(F("ODOM,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(getLeftDistanceM(), 6);
  Serial.print(',');
  Serial.print(getRightDistanceM(), 6);
  Serial.print(',');
  Serial.print(getOdomTheta(), 6);
  Serial.print(',');
  Serial.print(getImuGyroZ_radps(), 6);
  Serial.print(',');
  Serial.print(leftPulses);
  Serial.print(',');
  Serial.print(rightPulses);
  Serial.print(',');
  Serial.print(getImuMagX_uT(), 6);
  Serial.print(',');
  Serial.print(getImuMagY_uT(), 6);
  Serial.print(',');
  Serial.print(getImuMagZ_uT(), 6);
  Serial.print(',');
  Serial.print(getImuAccelX_mps2(), 6);
  Serial.print(',');
  Serial.print(getImuAccelY_mps2(), 6);
  Serial.print(',');
  Serial.print(getImuAccelZ_mps2(), 6);
  Serial.print(',');
  Serial.println(isImuReady() ? 1 : 0);
#endif
}

static void logPinTestState()
{
  Serial.print(F("[PINTEST] STBY="));
  Serial.print(g_pinTest.stby);
  Serial.print(F(" AIN1="));
  Serial.print(g_pinTest.a1);
  Serial.print(F(" AIN2="));
  Serial.print(g_pinTest.a2);
  Serial.print(F(" PWMA="));
  Serial.print(g_pinTest.pwmA);
  Serial.print(F(" BIN1="));
  Serial.print(g_pinTest.b1);
  Serial.print(F(" BIN2="));
  Serial.print(g_pinTest.b2);
  Serial.print(F(" PWMB="));
  Serial.println(g_pinTest.pwmB);
}

static void applyPinTestState()
{
  digitalWrite(PIN_Motor_STBY, g_pinTest.stby ? HIGH : LOW);
  digitalWrite(PIN_Motor_AIN_1, g_pinTest.a1 ? HIGH : LOW);
  digitalWrite(PIN_Motor_AIN_2, g_pinTest.a2 ? HIGH : LOW);
  digitalWrite(PIN_Motor_BIN_1, g_pinTest.b1 ? HIGH : LOW);
  digitalWrite(PIN_Motor_BIN_2, g_pinTest.b2 ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMA, g_pinTest.pwmA);
  analogWrite(PIN_Motor_PWMB, g_pinTest.pwmB);
  logPinTestState();
}

static void setPinTestOutputs(bool active,
                              uint8_t stby, uint8_t a1, uint8_t a2, uint8_t pwmA,
                              uint8_t b1, uint8_t b2, uint8_t pwmB)
{
  g_pinTest.active = active;
  g_pinTest.stby = stby;
  g_pinTest.a1 = a1;
  g_pinTest.a2 = a2;
  g_pinTest.pwmA = pwmA;
  g_pinTest.b1 = b1;
  g_pinTest.b2 = b2;
  g_pinTest.pwmB = pwmB;
  applyPinTestState();
}

static void beginPinTestMode()
{
  g_mode = MODE_STANDBY;
  lastCmdMs = millis();
}

static void exitPinTestMode()
{
  if (!g_pinTest.active)
  {
    return;
  }

  setPinTestOutputs(false, 0, 0, 0, 0, 0, 0, 0);
  Serial.println(F("[PINTEST] OFF"));
}

static void printPinTestHelp()
{
  Serial.println(F("PINTEST commands:"));
  Serial.println(F("  PINTEST,SHOW"));
  Serial.println(F("  PINTEST,OFF"));
  Serial.println(F("  PINTEST,A_FWD / A_REV / B_FWD / B_REV"));
  Serial.println(F("  PINTEST,<STBY|AIN1|AIN2|PWMA|BIN1|BIN2|PWMB>,<value>"));
  Serial.println(F("  Example: PINTEST,AIN1,1"));
  Serial.println(F("  Example: PINTEST,PWMA,255"));
}

static bool handlePinTestCommand(const String &line)
{
  String cmd = line;
  cmd.trim();
  cmd.toUpperCase();

  if (!cmd.startsWith("PINTEST"))
  {
    return false;
  }

  if (cmd == "PINTEST" || cmd == "PINTEST,HELP")
  {
    printPinTestHelp();
    return true;
  }

  if (!cmd.startsWith("PINTEST,"))
  {
    Serial.println(F("[PINTEST] Invalid command"));
    printPinTestHelp();
    return true;
  }

  String body = cmd.substring(8);
  body.trim();

  if (body == "SHOW")
  {
    logPinTestState();
    return true;
  }

  if (body == "OFF")
  {
    exitPinTestMode();
    return true;
  }

  beginPinTestMode();

  if (body == "A_FWD")
  {
    setPinTestOutputs(true, 1, 1, 0, 255, 0, 0, 0);
    return true;
  }
  if (body == "A_REV")
  {
    setPinTestOutputs(true, 1, 0, 1, 255, 0, 0, 0);
    return true;
  }
  if (body == "B_FWD")
  {
    setPinTestOutputs(true, 1, 0, 0, 0, 1, 0, 255);
    return true;
  }
  if (body == "B_REV")
  {
    setPinTestOutputs(true, 1, 0, 0, 0, 0, 1, 255);
    return true;
  }

  const int comma = body.indexOf(',');
  if (comma <= 0)
  {
    Serial.println(F("[PINTEST] Invalid command"));
    printPinTestHelp();
    return true;
  }

  String pinName = body.substring(0, comma);
  const int value = body.substring(comma + 1).toInt();

  if (pinName == "STBY")
    g_pinTest.stby = (value != 0) ? 1 : 0;
  else if (pinName == "AIN1")
    g_pinTest.a1 = (value != 0) ? 1 : 0;
  else if (pinName == "AIN2")
    g_pinTest.a2 = (value != 0) ? 1 : 0;
  else if (pinName == "BIN1")
    g_pinTest.b1 = (value != 0) ? 1 : 0;
  else if (pinName == "BIN2")
    g_pinTest.b2 = (value != 0) ? 1 : 0;
  else if (pinName == "PWMA")
    g_pinTest.pwmA = constrain(value, 0, 255);
  else if (pinName == "PWMB")
    g_pinTest.pwmB = constrain(value, 0, 255);
  else
  {
    Serial.println(F("[PINTEST] Unknown pin"));
    printPinTestHelp();
    return true;
  }

  g_pinTest.active = true;
  applyPinTestState();
  return true;
}

// ---- Motor helpers ----
static void motorsStop()
{
  Motor.DeviceDriverSet_Motor_control(direction_back, 0,
                                      direction_back, 0, control_disable);
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
  Motor.DeviceDriverSet_Motor_control(direction_just, spd, // A = Right
                                      direction_back, spd, // B = Left
                                      control_enable);
}

static void motorsRight(uint8_t spd)
{
  setCommandSigns(+1, -1);
  Motor.DeviceDriverSet_Motor_control(direction_back, spd, // A = Right
                                      direction_just, spd, // B = Left
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
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (handlePinTestCommand(line))
    {
      return;
    }

    if (line.equalsIgnoreCase("STOP"))
    {
      exitPinTestMode();
      g_mode = MODE_STANDBY;
      motorsStop();
      lastCmdMs = millis();
      return;
    }

    if (line.startsWith("VEL,"))
    {
      exitPinTestMode();
      g_mode = MODE_DRIVE;

      int c1 = line.indexOf(',', 4);
      if (c1 > 0)
      {
        int L = line.substring(4, c1).toInt();
        int R = line.substring(c1 + 1).toInt();

        L = constrain(L, -255, 255);
        R = constrain(R, -255, 255);

        // ---------- new zero‑speed handling ----------
        if (L == 0 && R == 0)
        {
          motorsStop(); // disable driver → active brake / standby
          lastCmdMs = millis();
          return;
        }
        // ---------------------------------------------

        setCommandSigns((L >= 0) ? +1 : -1, (R >= 0) ? +1 : -1);

        // A = Right, B = Left
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
      exitPinTestMode();
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

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS)
  {
    if (g_mode != MODE_STANDBY)
    {
      g_mode = MODE_STANDBY;
      motorsStop();
    }
  }
}

void setup()
{
  Serial.begin(BAUD);

  setupImu();
  setupOdometry();

  Motor.DeviceDriverSet_Motor_Init();
  AppIRrecv.DeviceDriverSet_IRrecv_Init();

  g_mode = MODE_STANDBY;
  motorsStop();
  lastCmdMs = millis();
}

void loop()
{
  updateImu();
  updateOdometry();

  uint8_t btn;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&btn))
  {
    const uint32_t now = millis();
    const uint32_t dtMs = (lastIrBtnMs == 0) ? 0 : (now - lastIrBtnMs);
    const bool isRepeat = (btn == lastIrBtn) && (dtMs < CMD_TIMEOUT_MS);
    const bool accepted = (!isRepeat || btn == 5);

    lastIrBtn = btn;
    lastIrBtnMs = now;

    if (accepted)
    {
      handleButton(btn);
      if (isMotionButton(btn))
      {
        lastCmdMs = now;
      }
    }
  }

  handleSerialJetson();
  sendOdomPacket();
}
