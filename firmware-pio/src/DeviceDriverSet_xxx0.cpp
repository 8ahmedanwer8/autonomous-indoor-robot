#include "DeviceDriverSet_xxx0.h"
#include "telemetry_config.h"
// #include "PinChangeInt.h"
#include <avr/wdt.h>

static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}
/*RBG LED*/
static uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds[Number] = colour;
    FastLED.show();
    delay_xxx(Duration);
  }
}
// void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness)
// {
//   FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
//   FastLED.setBrightness(set_Brightness);
// }
#if _Test_DeviceDriverSet
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Test(void)
{
  leds[0] = CRGB::White;
  FastLED.show();
  delay_xxx(50);
  leds[1] = CRGB::Red;
  FastLED.show();
  delay_xxx(50);
  DeviceDriverSet_RBGLED_xxx(50 /*Duration*/, 5 /*Traversal_Number*/, CRGB::Black);
}
#endif

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b)
{
  if (LED_s > NUM_LEDS)
    return;
  if (LED_s == NUM_LEDS)
  {
    FastLED.showColor(Color(r, g, b));
  }
  else
  {
    leds[LED_s] = Color(r, g, b);
  }
  FastLED.show();
}

/*Key*/
uint8_t DeviceDriverSet_Key::keyValue = 0;

static void attachPinChangeInterrupt_GetKeyValue(void)
{
  DeviceDriverSet_Key Key;
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;
  if ((millis() - keyValue_time) > 500)
  {
    keyValue_temp++;
    keyValue_time = millis();
    if (keyValue_temp > keyValue_Max)
    {
      keyValue_temp = 0;
    }
    Key.keyValue = keyValue_temp;
  }
}
void DeviceDriverSet_Key::DeviceDriverSet_Key_Init(void)
{
  pinMode(PIN_Key, INPUT_PULLUP);
  // attachPinChangeInterrupt(PIN_Key, attachPinChangeInterrupt_GetKeyValue, FALLING);
  attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Key::DeviceDriverSet_Key_Test(void)
{
  Serial.println(DeviceDriverSet_Key::keyValue);
}
#endif

void DeviceDriverSet_Key::DeviceDriverSet_key_Get(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue;
}

/*ITR20001 Detection*/
bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Test(void)
{
  Serial.print("\tL=");
  Serial.print(analogRead(PIN_ITR20001xxxL));

  Serial.print("\tM=");
  Serial.print(analogRead(PIN_ITR20001xxxM));

  Serial.print("\tR=");
  Serial.println(analogRead(PIN_ITR20001xxxR));
}
#endif

/*Voltage Detection*/
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Init(void)
{
  pinMode(PIN_Voltage, INPUT);
  // analogReference(INTERNAL);
}
float DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_getAnalogue(void)
{
  // float Voltage = ((analogRead(PIN_Voltage) * 5.00 / 1024) * 7.67); //7.66666=((10 + 1.50) / 1.50)
  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08); // Compensation 8%
  // return (analogRead(PIN_Voltage) * 5.00 / 1024) * ((10 + 1.50) / 1.50); //Read voltage value
  return Voltage;
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Test(void)
{
  // float Voltage = ((analogRead(PIN_Voltage) * 5.00 / 1024) * 7.67); //7.66666=((10 + 1.50) / 1.50)
  float Voltage = (analogRead(PIN_Voltage) * 0.0375); // 7.66666=((10 + 1.50) / 1.50)
  Voltage = Voltage + (Voltage * 0.08);               // Compensation 8%
  // Serial.println(analogRead(PIN_Voltage) * 4.97 / 1024);
  Serial.println(Voltage);
}
#endif
/*Motor control*/

#if TELEMETRY_MODE == TELEMETRY_MODE_TELEPLOT_DEBUG
static void logMotorState(uint8_t a1, uint8_t a2, uint8_t pwmA,
                          uint8_t b1, uint8_t b2, uint8_t pwmB,
                          uint8_t stby)
{
  Serial.print(F("[MOTOR] A1="));
  Serial.print(a1);
  Serial.print(F(" A2="));
  Serial.print(a2);
  Serial.print(F(" PA="));
  Serial.print(pwmA);
  Serial.print(F(" B1="));
  Serial.print(b1);
  Serial.print(F(" B2="));
  Serial.print(b2);
  Serial.print(F(" PB="));
  Serial.print(pwmB);
  Serial.print(F(" STBY="));
  Serial.println(stby);
}
#else
static void logMotorState(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t)
{
}
#endif

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_AIN_2, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_2, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{
  // A...Right
  // B...Left
  digitalWrite(PIN_Motor_STBY, HIGH);

  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_AIN_2, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_2, LOW);
  analogWrite(PIN_Motor_PWMB, 100);
  delay_xxx(1000);

  digitalWrite(PIN_Motor_STBY, LOW);
  delay_xxx(1000);
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_AIN_2, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_2, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);

  delay_xxx(1000);
}
#endif

/*
 Motor_control：AB / movement direction and speed
*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, // Group A motor parameters
                                                          boolean direction_B, uint8_t speed_B, // Group B motor parameters
                                                          boolean controlED                     // AB enable setting (true)
                                                          )                                     // Motor control
{
  uint8_t a1 = LOW;
  uint8_t a2 = LOW;
  uint8_t b1 = LOW;
  uint8_t b2 = LOW;
  uint8_t pwmA = 0;
  uint8_t pwmB = 0;
  uint8_t stby = LOW;

  if (controlED == control_enable) // Enable motot control？
  {
    stby = HIGH;
    // A...Right
    if (speed_A == 0)
    {
      a1 = LOW;
      a2 = LOW;
    }
    else if (direction_A == direction_just)
    {
      a1 = HIGH;
      a2 = LOW;
    }
    else
    {
      a1 = LOW;
      a2 = HIGH;
    }
    pwmA = speed_A;

    // B...Left
    if (speed_B == 0)
    {
      b1 = LOW;
      b2 = LOW;
    }
    else if (direction_B == direction_just)
    {
      b1 = HIGH;
      b2 = LOW;
    }
    else
    {
      b1 = LOW;
      b2 = HIGH;
    }
    pwmB = speed_B;
  }

  digitalWrite(PIN_Motor_STBY, stby);
  digitalWrite(PIN_Motor_AIN_1, a1);
  digitalWrite(PIN_Motor_AIN_2, a2);
  digitalWrite(PIN_Motor_BIN_1, b1);
  digitalWrite(PIN_Motor_BIN_2, b2);
  analogWrite(PIN_Motor_PWMA, pwmA);
  analogWrite(PIN_Motor_PWMB, pwmB);
  logMotorState(a1, a2, pwmA, b1, b2, pwmB, stby);
}

/*IRrecv*/
IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void)
{
  irrecv.enableIRIn(); // Enable infrared communication NEC
}
bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/)
{
  if (irrecv.decode(&results))
  {
    IR_PreMillis = millis();
    switch (results.value)
    {
    case /* constant-expression */ aRECV_upper:
    case /* constant-expression */ bRECV_upper:
      /* code */ *IRrecv_Get = 1;
      break;
    case /* constant-expression */ aRECV_lower:
    case /* constant-expression */ bRECV_lower:
      /* code */ *IRrecv_Get = 2;
      break;
    case /* constant-expression */ aRECV_Left:
    case /* constant-expression */ bRECV_Left:
      /* code */ *IRrecv_Get = 3;
      break;
    case /* constant-expression */ aRECV_right:
    case /* constant-expression */ bRECV_right:
      /* code */ *IRrecv_Get = 4;
      break;
    case /* constant-expression */ aRECV_ok:
    case /* constant-expression */ bRECV_ok:
      /* code */ *IRrecv_Get = 5;
      break;

    case /* constant-expression */ aRECV_1:
    case /* constant-expression */ bRECV_1:
      /* code */ *IRrecv_Get = 6;
      break;
    case /* constant-expression */ aRECV_2:
    case /* constant-expression */ bRECV_2:
      /* code */ *IRrecv_Get = 7;
      break;
    case /* constant-expression */ aRECV_3:
    case /* constant-expression */ bRECV_3:
      /* code */ *IRrecv_Get = 8;
      break;
    case /* constant-expression */ aRECV_4:
    case /* constant-expression */ bRECV_4:
      /* code */ *IRrecv_Get = 9;
      break;
    case /* constant-expression */ aRECV_5:
    case /* constant-expression */ bRECV_5:
      /* code */ *IRrecv_Get = 10;
      break;
    case /* constant-expression */ aRECV_6:
    case /* constant-expression */ bRECV_6:
      /* code */ *IRrecv_Get = 11;
      break;
    case /* constant-expression */ aRECV_7:
    case /* constant-expression */ bRECV_7:
      /* code */ *IRrecv_Get = 12;
      break;
    case /* constant-expression */ aRECV_8:
    case /* constant-expression */ bRECV_8:
      /* code */ *IRrecv_Get = 13;
      break;
    case /* constant-expression */ aRECV_9:
    case /* constant-expression */ bRECV_9:
      /* code */ *IRrecv_Get = 14;
      break;
    default:
      // *IRrecv_Get = 5;
      irrecv.resume();
      return false;
      break;
    }
    irrecv.resume();
    return true;
  }
  else
  {
    return false;
  }
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Test(void)
{
  if (irrecv.decode(&results))
  {
    Serial.print("IRrecv_Test:");
    Serial.println(results.value);
    irrecv.resume();
  }
}
#endif
