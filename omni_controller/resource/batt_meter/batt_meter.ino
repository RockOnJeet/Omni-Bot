const float ADC_REF = 5.0; // Arduino reference voltage
const int ADC_MAX = 1023;

int pinCurrent = A1;
int pinVoltage = A0;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  int rawI = analogRead(pinCurrent);
  int rawV = analogRead(pinVoltage);

  // Assuming a 10:1 voltage divider for current sensing
  float vCurrent = rawI * ADC_REF / ADC_MAX * 100;
  float vBatt = rawV * ADC_REF / ADC_MAX * 10;

  Serial.print(">Vcurrent:");
  Serial.print(vCurrent);
  Serial.print(",Vbatt:");
  Serial.println(vBatt);

  delay(200);
}
