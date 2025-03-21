// IMU
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];
byte packet[64];

// Constants
#define REFRESH_RATE 65.0 // Hz
#define TICKS_PER_REV 400

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
String data;

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
  Serial.setTimeout(3);

  // Set up IMU
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  mpu.initialize();
  if (mpu.testConnection() == 0) {
    Serial.println(F("MPU6050 connection failed"));
    while(true) delay(1000);
  }
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    Serial.print(F("DMP initialization failed: "));
    Serial.println(devStatus);
    while(true) delay(1000);
  }
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println(F("Ready"));
  mpu.setDMPEnabled(true);

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

  // Reserve space for encoder data
  data.reserve(25);

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
    data = Serial.readStringUntil('\n');
    if (data.startsWith("[") && data.endsWith("]")) {
      data.remove(0, 1);
      data.remove(data.length() - 1, 1);  // Remove '[' and ']'
      
      // Parse data for 3 values
      int fwdPWM = data.substring(0, data.indexOf("|")).toInt();
      int leftPWM = data.substring(data.indexOf("|") + 1, data.lastIndexOf("|")).toInt();
      int rightPWM = data.substring(data.lastIndexOf("|") + 1).toInt();

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
    angles[i] = (encTicks[i] * 2.0 * PI) / TICKS_PER_REV;
    Serial.print(angles[i]); Serial.print('|');
  }

  // Calucalte Velocites
  for (byte i = 0; i < 2; i++) {
    velocity[i] = (angles[i] - prevAngles[i]) * REFRESH_RATE;  // rad/s
    prevAngles[i] = angles[i];
    Serial.print(velocity[i]);
    Serial.print('|');
  }

  // Calulate Yaw
  if (mpu.dmpGetCurrentFIFOPacket(packet)) {
    mpu.dmpGetQuaternion(&q, packet);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(-ypr[0]);
  }
  Serial.print("}\n");

  delay(1000 / REFRESH_RATE); // Adjust as needed
  // delay(100); // Prevents serial buffer overflow
}
