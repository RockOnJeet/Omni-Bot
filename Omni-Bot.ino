#include <MD_REncoder.h>

class Interpolator {
 private:
  byte iterator = 0;
  byte _target = 0;
  unsigned long _time = 0;
  unsigned long _ms = 0;
  float slope = 0;

 public:
  Interpolator() {}

  void init(byte start = 0, byte target = 0, unsigned long ms = 0) {
    if (_target == 0) {  // first init
      _target = target;
      iterator = start;
      _time = millis() + ms;
      _ms = millis();
      slope = (float)(_target - iterator) / (float)(_time - _ms);
    }
  }

  void update(byte target, unsigned long ms = 0) {
    if (ms > 0) {
      reset();
      init(iterator, target, ms);
    } else {
      _target = target;
    }
  }

  byte getValue() {
    return millis() > _time ? _target : (slope * (millis() - _ms));
  }

  void reset() {
    _target = 0;
    iterator = 0;
    _time = 0;
    _ms = 0;
    slope = 0;
  }
};

Interpolator mainRamp, subRamp;

typedef struct motor {
  byte pwmPin;
  byte dirPin;
};

const motor motors[3] = {{5, 44}, {4, 26}, {9, 37}};
const uint16_t accelTime = 2000;

volatile int xCounter, yCounter, X, Y;

MD_REncoder encoder[2] = {MD_REncoder(2, 3), MD_REncoder(19, 18)};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].pwmPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
  }
  encoder[0].begin();
  encoder[1].begin();
  Serial.println("Ready");
}

void loop() {
  getEncodeData();
}

void getEncodeData() {
  byte x = encoder[0].read();
  byte y = encoder[1].read();
  if (x != DIR_NONE) {
    // Serial.print(x == DIR_CW ? "\n+1x" : "\n-1x");
    xCounter += x == DIR_CW ? 1 : -1;
    // if (xCounter > 400 || xCounter < -400) {
      while (xCounter > 7) {
        // X++;
        move('L');
        // delay(xCounter + 100);
        readEncoder();
      }
      while (xCounter < -7) {
        // X--;
        move('R');
        // delay(-xCounter + 100);
        readEncoder();
      }
        move('S');
      Serial.print("XRevs: ");
      Serial.println(xCounter);
      // xCounter = 0;
    // }
  }
  if (y != DIR_NONE) {
    // Serial.print(y == DIR_CW ? "\n+1y" : "\n-1y");
    yCounter += y == DIR_CW ? 1 : -1;
    // if (yCounter > 400 || yCounter < -400) {
      while (yCounter > 7) {
        // Y++;
        move('B');
        // delay(yCounter + 100);
        readEncoder();
      }
      while (yCounter < -7) {
        // Y--;
        move('F');
        // delay(-yCounter + 100);
        readEncoder();
      }
        move('S');
      Serial.print("YRevs: ");
      Serial.println(yCounter);
      // yCounter = 0;
    // }
  }
}

void readEncoder() {
  byte x = encoder[0].read();
  byte y = encoder[1].read();
  if (x != DIR_NONE) {
    xCounter += x == DIR_CW ? 1 : -1;
  }
  if (y != DIR_NONE) {
    yCounter += y == DIR_CW ? 1 : -1;
  }
}

void move(char c) {
  switch (c) {
    case 'F':
      mainRamp.init(0, 255, accelTime);
      // Serial.print(mainRamp.getValue());
      digitalWrite(motors[2].dirPin, HIGH);
      digitalWrite(motors[1].dirPin, LOW);
      for (byte i = 1; i < 3; i++) {
        analogWrite(motors[i].pwmPin, mainRamp.getValue());
      }
      break;
    case 'B':
      mainRamp.init(0, 255, accelTime);
      // Serial.print(mainRamp.getValue());
      digitalWrite(motors[2].dirPin, LOW);
      digitalWrite(motors[1].dirPin, HIGH);
      for (byte i = 1; i < 3; i++) {
        analogWrite(motors[i].pwmPin, mainRamp.getValue());
      }
      break;
    case 'L':
      mainRamp.init(0, 200, accelTime);
      subRamp.init(0, 255, accelTime);
      subRamp.update(180);
      // Serial.print(mainRamp.getValue());
      digitalWrite(motors[0].dirPin, HIGH);
      analogWrite(motors[0].pwmPin, mainRamp.getValue());
      for (byte i = 1; i < 3; i++) {
        digitalWrite(motors[i].dirPin, LOW);
        analogWrite(motors[i].pwmPin, subRamp.getValue());
      }
      break;
    case 'R':
      mainRamp.init(0, 200, accelTime);
      subRamp.init(0, 255, accelTime);
      subRamp.update(180);
      // Serial.print(mainRamp.getValue());
      digitalWrite(motors[0].dirPin, LOW);
      analogWrite(motors[0].pwmPin, mainRamp.getValue());
      for (byte i = 1; i < 3; i++) {
        digitalWrite(motors[i].dirPin, HIGH);
        analogWrite(motors[i].pwmPin, subRamp.getValue());
      }
      break;
    case 'S':
      // Serial.print(c);
      mainRamp.reset();
      subRamp.reset();
      for (byte i = 0; i < 3; i++) {
        digitalWrite(motors[i].dirPin, LOW);
        analogWrite(motors[i].pwmPin, 0);
      }
      break;
  }
}

