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

typedef struct motor {
  byte pwmPin;
  byte dirPin;
};

typedef enum { xy = -1, x, y } axis;

const motor motors[3] = {{5, 44}, {4, 26}, {9, 37}};
const uint16_t accelTime = 1000;
const byte ERR_THR = 15;

volatile int Offset[2] = {0, 0}, Origin[2] = {0, 0};
int errorAxis = xy;

MD_REncoder encoder[2] = {MD_REncoder(2, 3), MD_REncoder(19, 18)};
Interpolator fwdRamp, turnRamp;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  while (!Serial && !Serial2)
    ;
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].pwmPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
  }
  encoder[x].begin();
  encoder[y].begin();
  Serial.println("Ready");
}

void loop() {
  static char prevC = 'S';
  if (Serial2.available()) {
    char c = Serial2.read();
    if ((c == 'F' || c == 'B') && prevC != c) {
      errorAxis = x;
      getEncoderData(x);
      Origin[x] = Offset[x];
      prevC = c;
    } else if ((c == 'L' || c == 'R') && prevC != c) {
      errorAxis = y;
      getEncoderData(y);
      Origin[y] = Offset[y];
      prevC = c;
    } else if ((c == 'S') && prevC != c) {
      move('S');
      errorAxis = xy;
      getEncoderData(x);
      getEncoderData(y);
      Origin[x] = Offset[x];
      Origin[y] = Offset[y];
      prevC = c;
    }
    if (c != 'S')
    move(c);
  }

  getEncoderData(errorAxis);
  correctError(errorAxis);
}

void move(char c) {
  Serial.println(c);
  switch (c) {
    case 'F':
      fwdRamp.init(0, 255, accelTime);
      // Serial.print(fwdRamp.getValue());
      digitalWrite(motors[2].dirPin, HIGH);
      digitalWrite(motors[1].dirPin, LOW);
      for (byte i = 1; i < 3; i++) {
        analogWrite(motors[i].pwmPin, fwdRamp.getValue());
      }
      break;
    case 'B':
      fwdRamp.init(0, 255, accelTime);
      // Serial.print(fwdRamp.getValue());
      digitalWrite(motors[2].dirPin, LOW);
      digitalWrite(motors[1].dirPin, HIGH);
      for (byte i = 1; i < 3; i++) {
        analogWrite(motors[i].pwmPin, fwdRamp.getValue());
      }
      break;
    case 'L':
      turnRamp.init(0, 255, accelTime);
      // Serial.print(fwdRamp.getValue());
      digitalWrite(motors[0].dirPin, HIGH);
      analogWrite(motors[0].pwmPin, turnRamp.getValue());
      for (byte i = 1; i < 3; i++) {
        digitalWrite(motors[i].dirPin, LOW);
        analogWrite(motors[i].pwmPin, ((byte)turnRamp.getValue() / sqrt(2)));
      }
      break;
    case 'R':
      turnRamp.init(0, 255, accelTime);
      // Serial.print(fwdRamp.getValue());
      digitalWrite(motors[0].dirPin, LOW);
      analogWrite(motors[0].pwmPin, turnRamp.getValue());
      for (byte i = 1; i < 3; i++) {
        digitalWrite(motors[i].dirPin, HIGH);
        analogWrite(motors[i].pwmPin, ((byte)turnRamp.getValue() / sqrt(2)));
      }
      break;
    case 'S':
      // Serial.print(c);
      fwdRamp.reset();
      turnRamp.reset();
      for (byte i = 0; i < 3; i++) {
        digitalWrite(motors[i].dirPin, LOW);
        analogWrite(motors[i].pwmPin, 0);
      }
      break;
    case 'Q':
      turnRamp.init(0, 255, accelTime);
      digitalWrite(motors[0].dirPin, HIGH);
      analogWrite(motors[0].pwmPin, turnRamp.getValue());
      break;
    case 'E':
      turnRamp.init(0, 255, accelTime);
      digitalWrite(motors[0].dirPin, LOW);
      analogWrite(motors[0].pwmPin, turnRamp.getValue());
      break;
    case 'C':
      turnRamp.reset();
      digitalWrite(motors[0].dirPin, LOW);
      analogWrite(motors[0].pwmPin, 0);
      break;
  }
}

void getEncoderData(int axis) {
  if (axis == xy) {
    getEncoderData(x);
    getEncoderData(y);
    return;
  }
  byte _axisData = encoder[axis].read();
  if (_axisData != DIR_NONE) {
    Offset[axis] += _axisData == DIR_CW ? 1 : -1;
  }
  // Serial.print(axis);
  // Serial.print(": ");
  // Serial.println(Offset[axis]);
}

void correctError(int axis) {
  static bool isMoving[2] = {false, false};
  int _offSet[2] = {Offset[x], Offset[y]};
  if (axis == xy) {
    correctError(x);
    correctError(y);
    return;
  }
  _offSet[axis] = Origin[axis] - _offSet[axis];
  if (_offSet[axis] > ERR_THR) {
    isMoving[axis] = true;
    switch (axis) {
      case x:
        move('E');
        break;
      case y:
        // move('F');
        break;
    }
  } else if (_offSet[axis] < -ERR_THR) {
    isMoving[axis] = true;
    switch (axis) {
      case x:
        move('Q');
        break;
      case y:
        // move('B');
        break;
    }
  } else {
    if (isMoving[axis]) {
      move('C');
      isMoving[axis] = false;
    }
    // move('S');
  } 
}
