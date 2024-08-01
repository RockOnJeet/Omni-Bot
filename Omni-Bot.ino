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

const motor motors[3] = {{5, 44}, {4, 26}, {9, 37}};
const uint16_t accelTime = 2000;

volatile int xCounter, yCounter;

Interpolator mainRamp, subRamp;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].pwmPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
  }
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), xFall, RISING);
  attachInterrupt(digitalPinToInterrupt(3), xRise, RISING);
  attachInterrupt(digitalPinToInterrupt(18), yRise, RISING);
  attachInterrupt(digitalPinToInterrupt(19), yFall, RISING);
}

void loop() {
  if (Serial2.available() > 0) {
    char c = Serial2.read();
    move(c);
    Serial.print(xCounter);
    Serial.print(",");
    Serial.println(yCounter);
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

void xRise() {
  if (digitalRead(2) == LOW) {
    // Serial.print("xRise");
    xCounter++;
  }
}

void xFall() {
  if (digitalRead(3) == LOW) {
    // Serial.print("xFall");
    xCounter--;
  }
}

void yRise() {
  if (digitalRead(19) == LOW) {
    // Serial.print("yRise");
    yCounter++;
  }
}

void yFall() {
  if (digitalRead(18) == LOW) {
    // Serial.print("yFall");
    yCounter--;
  }
}
