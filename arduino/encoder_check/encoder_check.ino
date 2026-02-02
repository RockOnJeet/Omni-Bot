// Pins
const byte EncoderX[2] = {2, 4}; // X encoder pins
const byte EncoderY[2] = {3, 5}; // Y encoder pins

// Variables
volatile long encTicks[2];

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
  while(!Serial);

  // Set up encoders
  for (byte i = 0; i < 2; i++) {
    pinMode(EncoderX[i], INPUT_PULLUP);
    pinMode(EncoderY[i], INPUT_PULLUP);
  }

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(EncoderX[0]), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderY[0]), rightEncoderISR, RISING);
}

void loop() {
  Serial.print(encTicks[0]); Serial.print('\t');
  Serial.println(encTicks[1]);
}
