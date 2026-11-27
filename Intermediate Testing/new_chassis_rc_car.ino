// ================= L298N MOTOR PINS =================
const int IN1 = 8;   // Left Motor Forward
const int IN2 = 9;   // Left Motor Backward
const int IN3 = 10;  // Right Motor Forward
const int IN4 = 11;  // Right Motor Backward

// ================= POLOLU ENCODER PINS =================
// Channel A MUST be on hardware interrupt pins 2 and 3
const int LEFT_ENC_A  = 2;
const int LEFT_ENC_B  = 4;
const int RIGHT_ENC_A = 3;
const int RIGHT_ENC_B = 5;

// Volatile counters updated inside hardware interrupts
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// ================= CALIBRATION (TUNE THESE FOR YOUR GEAR RATIO) =================
// Example: A 100:1 Pololu motor with 12 CPR encoder = 1,200 ticks per wheel revolution
const long TICKS_PER_CELL = 1200;  // Ticks needed to move forward 1 grid cell
const long TICKS_PER_90_TURN = 580; // Ticks needed per wheel to pivot 90 degrees

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins (INPUT_PULLUP prevents floating signals on open-collector Hall sensors)
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach hardware interrupts on RISING edge of Channel A
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, RISING);

  stopCar();
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'F':
        moveDistance(TICKS_PER_CELL, true);
        break;
      case 'B':
        moveDistance(TICKS_PER_CELL, false);
        break;
      case 'L':
        pivotTurn(TICKS_PER_90_TURN, true);
        break;
      case 'R':
        pivotTurn(TICKS_PER_90_TURN, false);
        break;
      case 'S':
        stopCar();
        break;
    }
  }
}

// ================= INTERRUPT SERVICE ROUTINES =================
void readLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void readRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

// ================= ENCODER-GUIDED MOTION =================
void moveDistance(long targetTicks, bool forward) {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  if (forward) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }

  // Run until average distance of both wheels reaches targetTicks
  while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {
    // Allow emergency stop via Serial mid-move
    if (Serial.available() > 0 && Serial.read() == 'S') break;
  }

  stopCar();
}

void pivotTurn(long targetTicks, bool turnLeft) {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  if (turnLeft) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }

  while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {
    if (Serial.available() > 0 && Serial.read() == 'S') break;
  }

  stopCar();
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
