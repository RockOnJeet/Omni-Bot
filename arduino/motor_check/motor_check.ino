const byte MotorF[2] = {9, 6};  // Front motor pins (PWM, DIR)
const byte MotorL[2] = {10, 7}; // Left motor pins (PWM, DIR)
const byte MotorR[2] = {11, 8}; // Right motor pins (PWM, DIR)

String data;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  for (byte i = 0; i < 2; i++)
  {
    pinMode(MotorF[i], OUTPUT);
    pinMode(MotorL[i], OUTPUT);
    pinMode(MotorR[i], OUTPUT);
  }
  Serial.println("Ready");
}

void loop()
{
  // Parse PWM commands (non-blocking, can happen anytime)
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('\n');
    data.trim(); // Remove any leading/trailing whitespace
    if (data.startsWith("[") && data.endsWith("]"))
    {
      data.remove(0, 1);
      data.remove(data.length() - 1, 1); // Remove '[' and ']'

      // Parse data for 3 values
      int fwdPWM = data.substring(0, data.indexOf("|")).toInt();
      int leftPWM =
          data.substring(data.indexOf("|") + 1, data.lastIndexOf("|")).toInt();
      int rightPWM = data.substring(data.lastIndexOf("|") + 1).toInt();

      // Set motor direction and speed
      digitalWrite(MotorR[1], fwdPWM > 0 ? HIGH : LOW);
      analogWrite(MotorF[0], abs(fwdPWM));

      digitalWrite(MotorL[1], leftPWM > 0 ? HIGH : LOW);
      analogWrite(MotorL[0], abs(leftPWM));

      digitalWrite(MotorF[1], rightPWM > 0 ? HIGH : LOW);
      analogWrite(MotorR[0], abs(rightPWM));

      Serial.print(F("PWM: "));
      Serial.print(fwdPWM);
      Serial.print(F(" "));
      Serial.print(leftPWM);
      Serial.print(F(" "));
      Serial.println(rightPWM);
      Serial.print(F("Dir: "));
      Serial.print(fwdPWM > 0 ? "AC" : "C");
      Serial.print(F(" "));
      Serial.print(leftPWM > 0 ? "AC" : "C");
      Serial.print(F(" "));
      Serial.println(rightPWM > 0 ? "AC" : "C");
    }
    while (Serial.available() > 0)
      Serial.read(); // Clear the buffer
  }
}