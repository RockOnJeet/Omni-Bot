// #include <SoftwareSerial.h>

// SoftwareSerial Serial(10, 11); // RX, TX pins for Bluetooth module

// Define the PWM and DIR pins for each motor

const int pwmPin1 = 9; // Replace with the actual PWM pin for motor 1
const int dirPin1 = 5; // Replace with the actual DIR pin for motor 1

const int pwmPin2 = 10; // Replace with the actual PWM pin for motor 2
const int dirPin2 = 6; // Replace with the actual DIR pin for motor 2

const int pwmPin3 = 11; // Replace with the actual PWM pin for motor 3
const int dirPin3 = 7; // Replace with the actual DIR pin for motor 3

void setup() {
  // Serial.begin(9600);
  Serial.begin(9600);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
   pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
   pinMode(pwmPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    // Control commands from Bluetooth
    switch (command) {
      case 'F': // Forward
        Serial.print(command);
        digitalWrite(dirPin1, HIGH);  
        analogWrite(pwmPin1, 0); 
        digitalWrite(dirPin2, LOW);  
        analogWrite(pwmPin2, 255); 
        digitalWrite(dirPin3, HIGH);  
        analogWrite(pwmPin3, 255); 
        break;
      case 'B': // Backward
        digitalWrite(dirPin1, HIGH);  
        analogWrite(pwmPin1, 0); 
        digitalWrite(dirPin2, HIGH);  
        analogWrite(pwmPin2, 225); 
        digitalWrite(dirPin3, LOW);  
        analogWrite(pwmPin3, 225); 
        Serial.print(command);
        break;
      case 'L': // Left
        Serial.print(command);
        digitalWrite(dirPin1, HIGH);  
        analogWrite(pwmPin1, 254); 
        digitalWrite(dirPin2, LOW);  
        analogWrite(pwmPin2, 126); 
        digitalWrite(dirPin3, LOW);  
        analogWrite(pwmPin3, 126); 
        break;
      case 'R':
        digitalWrite(dirPin1, LOW);
        analogWrite(pwmPin1, 254);
        digitalWrite(dirPin2, HIGH);
        analogWrite(pwmPin2, 126);
        digitalWrite(dirPin3, HIGH);
        analogWrite(pwmPin3, 126);
        Serial.print(command);
        break;
      case 'G':
        Serial.print(command);
        digitalWrite(dirPin1, HIGH);  
        analogWrite(pwmPin1, 255); 
        digitalWrite(dirPin2, LOW);  
        analogWrite(pwmPin2, 255); 
        digitalWrite(dirPin3, HIGH);  
        analogWrite(pwmPin3, 127.5); 
        break;
      case 'H':
        digitalWrite(dirPin1, HIGH);  
        analogWrite(pwmPin1, 255); 
        digitalWrite(dirPin2, HIGH);  
        analogWrite(pwmPin2, 127.5); 
        digitalWrite(dirPin3, LOW);  
        analogWrite(pwmPin3, 255); 
        Serial.print(command);
        break;
      case 'I':
        digitalWrite(dirPin1, LOW);  
        analogWrite(pwmPin1, 255); 
        digitalWrite(dirPin2, LOW);  
        analogWrite(pwmPin2, 127.5); 
        digitalWrite(dirPin3, HIGH);  
        analogWrite(pwmPin3, 255); 
        Serial.print(command);
        break;
      case 'J':
        digitalWrite(dirPin1, LOW);  
        analogWrite(pwmPin1, 255); 
        digitalWrite(dirPin2, HIGH);  
        analogWrite(pwmPin2, 255); 
        digitalWrite(dirPin3, LOW);  
        analogWrite(pwmPin3, 127.5); 
        Serial.print(command);
        break;
      case 'S': // Stop
        Serial.print(command);
        digitalWrite(dirPin1, HIGH);  
        analogWrite(pwmPin1, 0); 
        digitalWrite(dirPin2, LOW);  
        analogWrite(pwmPin2, 0); 
        digitalWrite(dirPin3, LOW);  
        analogWrite(pwmPin3, 0); 
        break;
    }
  }
}
