const uint8_t TEST_PIN = 2;

void printHelp()
{
  Serial.println(F("D2 pin probe"));
  Serial.println(F("Commands:"));
  Serial.println(F("  L or LOW   -> drive D2 low"));
  Serial.println(F("  H or HIGH  -> drive D2 high"));
  Serial.println(F("  S or SHOW  -> print current state"));
}

void drivePin(bool high)
{
  digitalWrite(TEST_PIN, high ? HIGH : LOW);
  Serial.print(F("D2="));
  Serial.println(high ? F("HIGH") : F("LOW"));
}

void setup()
{
  Serial.begin(115200);
  pinMode(TEST_PIN, OUTPUT);
  drivePin(false);
  printHelp();
}

void loop()
{
  if (!Serial.available())
  {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "L" || cmd == "LOW")
  {
    drivePin(false);
  }
  else if (cmd == "H" || cmd == "HIGH")
  {
    drivePin(true);
  }
  else if (cmd == "S" || cmd == "SHOW")
  {
    Serial.print(F("D2 reads back as "));
    Serial.println(digitalRead(TEST_PIN) ? F("HIGH") : F("LOW"));
  }
  else
  {
    printHelp();
  }
}
