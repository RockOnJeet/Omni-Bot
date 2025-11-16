#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Comment if Binary Output is desired
// #define OUT_YPR

MPU6050 mpu;

byte packet[64];
#ifdef OUT_YPR
Quaternion q;
VectorFloat gravity;
float ypr[3];
#else
int16_t ax, ay, az;
int16_t gx, gy, gz;
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties

  Serial.println(F("Initializing..."));
  mpu.initialize();
  if (mpu.testConnection() == 0) {
    Serial.println(F("MPU6050 connection failed"));
    while (true) delay(1000);
  }
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    Serial.print(F("DMP initialization failed: "));
    Serial.println(devStatus);
    while (true) delay(1000);
  }
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  // Serial.println(F("Connected! Current offsets:"));
  // mpu.PrintActiveOffsets();
  // delay(3000);
  mpu.setDMPEnabled(true);
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(packet)) {
#ifdef OUT_YPR
    mpu.dmpGetQuaternion(&q, packet);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(">Yaw:");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(",Pitch:");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(",Roll:");
    Serial.println(ypr[2] * 180 / M_PI);
#else
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.write((uint8_t)(ax >> 8));
    Serial.write((uint8_t)(ax & 0xFF));
    Serial.write((uint8_t)(ay >> 8));
    Serial.write((uint8_t)(ay & 0xFF));
    Serial.write((uint8_t)(az >> 8));
    Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gx >> 8));
    Serial.write((uint8_t)(gx & 0xFF));
    Serial.write((uint8_t)(gy >> 8));
    Serial.write((uint8_t)(gy & 0xFF));
    Serial.write((uint8_t)(gz >> 8));
    Serial.write((uint8_t)(gz & 0xFF));
#endif
  }

  delay(1000);
}
