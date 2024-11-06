#include <SoftwareSerial.h>

typedef enum {
  CW = 0,
  CCW = 1
} Direction;

class Motor {
  private:
    byte pwmPin;
    byte dirPin;
    byte motorPWM;
    Direction motorDirection;
  
  public:
    Motor(byte pwmPin, byte dirPin) {
      this->pwmPin = pwmPin;
      this->dirPin = dirPin;
      motorPWM = 0;
    }
  
    void setSpeed(byte PWM) {
      motorPWM = PWM;
      analogWrite(pwmPin, motorPWM);
    }

    void setDirection(bool direction) {
      digitalWrite(dirPin, (bool)direction);
    }

    void stop() {
      motorPWM = 0;
      analogWrite(pwmPin, motorPWM);
    }
};

Motor motorA(5, 8);
Motor motorB(6, 9);
Motor motorC(7, 10);

SoftwareSerial BLE(7, 6); // RX, TX

void setup() {
  Serial.begin(115200);
  BLE.begin(115200);
  Serial.println("Begin!");
}

void loop() {
  if (BLE.available()) {
    char command = BLE.read();
    Serial.println(command);
    switch (command) {
      case 'F':
        motorA.stop();
        motorB.setDirection(CW);
        motorB.setSpeed(255);
        motorC.setDirection(CCW);
        motorC.setSpeed(255);
        break;
      case 'B':
        motorA.stop();
        motorB.setDirection(CCW);
        motorB.setSpeed(255);
        motorC.setDirection(CW);
        motorC.setSpeed(255);
        break;
      case 'L':
        motorA.setDirection(CW);
        motorA.setSpeed(255);
        motorB.setDirection(CW);
        motorB.setSpeed(255);
        motorC.setDirection(CW);
        motorC.setSpeed(255);
        break;
      case 'R':
        motorA.setDirection(CCW);
        motorA.setSpeed(255);
        motorB.setDirection(CCW);
        motorB.setSpeed(255);
        motorC.setDirection(CCW);
        motorC.setSpeed(255);
        break;
      default:
        motorA.stop();
        motorB.stop();
        motorC.stop();
        break;
    }
  }
}
