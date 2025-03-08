// Constants
#define REFRESH_RATE 30 // Hz

// Pins
const byte EncoderX[2] = {2, 4}; // X encoder pins
const byte EncoderY[2] = {3, 5}; // Y encoder pins
// const byte MotorL[3] = {6, 7, 8}; // Left motor pins (PWMB, IN3, IN4)
// const byte MotorR[3] = {9, 10, 11}; // Right motor pins (PWMA, IN1, IN2)

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
  // for (byte i = 0; i < 3; i++) {
  //   pinMode(MotorL[i], OUTPUT);
  //   pinMode(MotorR[i], OUTPUT);
  // }

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
  // Check for serial input
  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');
  //   if (input.startsWith("CMD")) {
  //     input.remove(0, 4); // Remove "CMD,"
  //     int commaIndex = input.indexOf(',');
  //     float leftSpeed = input.substring(0, commaIndex).toFloat();
  //     float rightSpeed = input.substring(commaIndex + 1).toFloat();

  //     // Map leftSpeed and rightSpeed to motor control values
  //     int leftPWM = constrain(map(leftSpeed, -10, 10, -255, 255), -255, 255);
  //     int rightPWM = constrain(map(rightSpeed, -10, 10, -255, 255), -255, 255);

  //     // Set motor direction and speed
  //     if (leftPWM <= 0) {
  //       digitalWrite(MotorL[1], HIGH);
  //       digitalWrite(MotorL[2], LOW);
  //     } else {
  //       digitalWrite(MotorL[1], LOW);
  //       digitalWrite(MotorL[2], HIGH);
  //     }
  //     analogWrite(MotorL[0], abs(leftPWM));

  //     if (rightPWM <= 0) {
  //       digitalWrite(MotorR[1], HIGH);
  //       digitalWrite(MotorR[2], LOW);
  //     } else {
  //       digitalWrite(MotorR[1], LOW);
  //       digitalWrite(MotorR[2], HIGH);
  //     }
  //     analogWrite(MotorR[0], abs(rightPWM));
  //   }
  // }

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
