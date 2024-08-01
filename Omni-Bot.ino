#include <MD_REncoder.h>

volatile int xCounter, yCounter, X, Y;

MD_REncoder encoder[2] = {MD_REncoder(2, 3), MD_REncoder(19, 18)};

void setup() {
  Serial.begin(115200);
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
    if (xCounter > 400 || xCounter < -400) {
      if (xCounter > 400)
        X++;
      else
        X--;
      Serial.print("XRevs: ");
      Serial.println(X);
      xCounter = 0;
    }
  }
  if (y != DIR_NONE) {
    // Serial.print(y == DIR_CW ? "\n+1y" : "\n-1y");
    yCounter += y == DIR_CW ? 1 : -1;
    if (yCounter > 400 || yCounter < -400) {
      if (yCounter > 400)
        Y++;
      else
        Y--;
      Serial.print("YRevs: ");
      Serial.println(Y);
      yCounter = 0;
    }
  }
}
