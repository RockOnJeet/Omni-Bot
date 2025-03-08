// Constants
#define REFRESH_RATE 30 // Hz

// Pins
const byte EncoderX[2] = {2, 4}; // X encoder pins
const byte EncoderY[2] = {3, 5}; // Y encoder pins
const byte MotorF[2] = {9, 6}; // Front motor pins (PWM, DIR)
const byte MotorL[2] = {10, 7}; // Left motor pins (PWM, DIR)
const byte MotorR[2] = {11, 8}; // Right motor pins (PWM, DIR)

// Variables
volatile long encTicks[2];
double velocity[2];
double angles[2], prevAngles[2];
unsigned long prevMillis;

// Interrupt Service Routines
void leftEncoderISR() {
  if (digitalRead(EncoderX[1]) == HIGH) {
    encTicks[0]++;
  } else {
    encTicks[0]--;
  }
}

void rightEncoderISR() {
  if (digitalRead(EncoderY[1]) != HIGH) {
    encTicks[1]++;
  } else {
    encTicks[1]--;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Set up motors
  for (byte i = 0; i < 2; i++) {
    pinMode(MotorF[i], OUTPUT);
    pinMode(MotorL[i], OUTPUT);
    pinMode(MotorR[i], OUTPUT);
  }

  // Set up encoders
  for (byte i = 0; i < 2; i++) {
    pinMode(EncoderX[i], INPUT_PULLUP);
    pinMode(EncoderY[i], INPUT_PULLUP);
  }

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(EncoderX[0]), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderY[0]), rightEncoderISR, RISING);

  // Initialize Sync
  while (Serial.read() != '?') {
    delay(10);
  }
  Serial.write('!');
  while (Serial.read() != '!') {
    delay(10);
  }
}

void loop() {
  // Parse PWM commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("[") && input.endsWith("]")) {
      input.remove(0, 1);
      input.remove(input.length() - 1, 1);  // Remove '[' and ']'
      
      // Parse input for 3 values
      int fwdPWM = input.substring(0, input.indexOf("|")).toInt();
      int leftPWM = input.substring(input.indexOf("|") + 1, input.lastIndexOf("|")).toInt();
      int rightPWM = input.substring(input.lastIndexOf("|") + 1).toInt();

      // Set motor direction and speed
      digitalWrite(MotorF[1], fwdPWM > 0 ? HIGH : LOW);
      analogWrite(MotorF[0], abs(fwdPWM));

      digitalWrite(MotorL[1], leftPWM > 0 ? HIGH : LOW);
      analogWrite(MotorL[0], abs(leftPWM));

      digitalWrite(MotorR[1], rightPWM > 0 ? HIGH : LOW);
      analogWrite(MotorR[0], abs(rightPWM));
    }
  }

  // Send encoder data
  Serial.write('{');
  // Calculate Angles
  for (byte i = 0; i < 2; i++) {
    angles[i] = (encTicks[i] * 2.0 * PI) / 140.0;
    Serial.print(angles[i]); Serial.print('|');
  }

  // Calucalte Velocites
  for (byte i = 0; i < 2; i++) {
    velocity[i] = (angles[i] - prevAngles[i]) * 1000 / (int)(1000 / REFRESH_RATE);  // rad/s
    prevAngles[i] = angles[i];
    Serial.print(velocity[i]);
    if (i < 1) Serial.print('|');
  }
  Serial.println('}');

  delay(1000 / REFRESH_RATE); // Adjust as needed
  // delay(100); // Prevents serial buffer overflow
}
