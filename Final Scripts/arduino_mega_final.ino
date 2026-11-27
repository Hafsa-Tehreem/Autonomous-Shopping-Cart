#include <Servo.h>

// ================= POLOLU DRIVE & ENCODER PINS =================
const int LEFT_FWD  = 22;
const int LEFT_BWD  = 23;
const int RIGHT_FWD = 24;
const int RIGHT_BWD = 25;

const int LEFT_ENC_A  = 2;  // Hardware Interrupt 0
const int LEFT_ENC_B  = 4;
const int RIGHT_ENC_A = 3;  // Hardware Interrupt 1
const int RIGHT_ENC_B = 5;

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// CALIBRATION: Tune for your Pololu gear ratio & wheel size
const long TICKS_PER_CELL    = 1200;
const long TICKS_PER_90_TURN = 580;

// ================= ROBOTIC ARM SERVOS =================
const int BASE_PIN     = 6;
const int SHOULDER_PIN = 7;
const int ELBOW_PIN    = 8;
const int GRIPPER_PIN  = 9;

Servo baseServo, shoulderServo, elbowServo, gripperServo;

int currBase = 90, currShoulder = 110, currElbow = 120, currGripper = 40;

// Calibrated Arm Poses
const int HOME_BASE = 90,  HOME_SHOULDER = 110, HOME_ELBOW = 120, GRIP_OPEN  = 40;
const int PICK_BASE = 90,  PICK_SHOULDER = 65,  PICK_ELBOW = 70,  GRIP_CLOSE = 115;
const int DROP_BASE = 180, DROP_SHOULDER = 85,  DROP_ELBOW = 95;

// ================= DWIN LCD ORDER BUFFER =================
String pendingOrder = "";
byte dwinFrame[9];

void setup() {
  Serial.begin(115200);   // USB Serial to Python
  Serial1.begin(115200);  // DWIN 4.3" HMI LCD (change to 9600 if DGUS CFG uses 9600)
  Serial2.begin(9600);    // ESP32-CAM Wi-Fi Bridge

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  stopMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, RISING);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);
  moveArmSmooth(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_OPEN, 12);
}

void loop() {
  // 1. Listen for DWIN HMI Touchscreen Selections (Serial1 -> USB Python)
  handleDwinTouch();

  // 2. Listen for Navigation & Arm Commands from ESP32-CAM (Serial2)
  if (Serial2.available() > 0) {
    char cmd = Serial2.read();
    executeCommand(cmd);
  }
}

// ================= DWIN DGUS FRAME PARSER =================
// Expected DGUS Return Key Values on VP 0x1000:
// 1 = milk, 2 = bread, 3 = apples, 4 = coffee, 99 = START MISSION
void handleDwinTouch() {
  if (Serial1.available() >= 9) {
    if (Serial1.read() == 0x5A && Serial1.read() == 0xA5) {
      dwinFrame[0] = 0x5A;
      dwinFrame[1] = 0xA5;
      for (int i = 2; i < 9; i++) {
        while (!Serial1.available());
        dwinFrame[i] = Serial1.read();
      }

      if (dwinFrame[3] == 0x83) { // VP Read/Return Command
        uint16_t keyVal = (dwinFrame[7] << 8) | dwinFrame[8];

        if (keyVal == 1) appendItem("milk");
        else if (keyVal == 2) appendItem("bread");
        else if (keyVal == 3) appendItem("apples");
        else if (keyVal == 4) appendItem("coffee");
        else if (keyVal == 99 && pendingOrder.length() > 0) {
          // Send complete shopping list over USB Serial to Python
          Serial.print("ORDER:");
          Serial.println(pendingOrder);
          pendingOrder = "";
        }
      }
    }
  }
}

void appendItem(String item) {
  if (pendingOrder.length() > 0) pendingOrder += ",";
  pendingOrder += item;
  Serial.print("LCD_SELECTED:");
  Serial.println(item);
}

// ================= COMMAND EXECUTION & ACK =================
void executeCommand(char cmd) {
  switch (cmd) {
    case 'F':
      moveDistance(TICKS_PER_CELL, true);
      Serial2.println("DONE");
      break;
    case 'B':
      moveDistance(TICKS_PER_CELL, false);
      Serial2.println("DONE");
      break;
    case 'L':
      pivotTurn(TICKS_PER_90_TURN, true);
      Serial2.println("DONE");
      break;
    case 'R':
      pivotTurn(TICKS_PER_90_TURN, false);
      Serial2.println("DONE");
      break;
    case 'P': // Pick scanned item & place in rear self-contained container
      pickAndStowItem();
      Serial2.println("DONE");
      break;
    case 'S':
      stopMotors();
      Serial2.println("DONE");
      break;
  }
}

// ================= ENCODER ISR & DRIVE FUNCTIONS =================
void readLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) leftTicks++;
  else leftTicks--;
}

void readRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) rightTicks++;
  else rightTicks--;
}

void moveDistance(long targetTicks, bool forward) {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  if (forward) {
    digitalWrite(LEFT_FWD, HIGH);  digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, HIGH); digitalWrite(RIGHT_BWD, LOW);
  } else {
    digitalWrite(LEFT_FWD, LOW);   digitalWrite(LEFT_BWD, HIGH);
    digitalWrite(RIGHT_FWD, LOW);  digitalWrite(RIGHT_BWD, HIGH);
  }

  while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {
    if (Serial2.available() > 0 && Serial2.peek() == 'S') {
      Serial2.read();
      break;
    }
  }
  stopMotors();
}

void pivotTurn(long targetTicks, bool turnLeft) {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  if (turnLeft) {
    digitalWrite(LEFT_FWD, LOW);   digitalWrite(LEFT_BWD, HIGH);
    digitalWrite(RIGHT_FWD, HIGH); digitalWrite(RIGHT_BWD, LOW);
  } else {
    digitalWrite(LEFT_FWD, HIGH);  digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, LOW);  digitalWrite(RIGHT_BWD, HIGH);
  }

  while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {
    if (Serial2.available() > 0 && Serial2.peek() == 'S') {
      Serial2.read();
      break;
    }
  }
  stopMotors();
}

void stopMotors() {
  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(LEFT_BWD, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BWD, LOW);
}

// ================= ROBOTIC ARM PICK & REAR STOW =================
void pickAndStowItem() {
  // 1. Reach toward shelf item with open gripper
  moveArmSmooth(PICK_BASE, PICK_SHOULDER, PICK_ELBOW, GRIP_OPEN, 12);
  delay(300);
  // 2. Grasp item
  moveArmSmooth(PICK_BASE, PICK_SHOULDER, PICK_ELBOW, GRIP_CLOSE, 12);
  delay(400);
  // 3. Lift clear of shelf
  moveArmSmooth(PICK_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_CLOSE, 12);
  delay(250);
  // 4. Rotate 180° to rear self-contained bin
  moveArmSmooth(DROP_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_CLOSE, 12);
  delay(250);
  // 5. Lower and release into rear container
  moveArmSmooth(DROP_BASE, DROP_SHOULDER, DROP_ELBOW, GRIP_CLOSE, 12);
  delay(250);
  moveArmSmooth(DROP_BASE, DROP_SHOULDER, DROP_ELBOW, GRIP_OPEN, 12);
  delay(400);
  // 6. Return to stowed home position
  moveArmSmooth(DROP_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_OPEN, 12);
  moveArmSmooth(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_OPEN, 12);
}

void moveArmSmooth(int tBase, int tShoulder, int tElbow, int tGripper, int stepDelay) {
  while (currBase != tBase || currShoulder != tShoulder ||
         currElbow != tElbow || currGripper != tGripper) {
    if (currBase < tBase) currBase++; else if (currBase > tBase) currBase--;
    if (currShoulder < tShoulder) currShoulder++; else if (currShoulder > tShoulder) currShoulder--;
    if (currElbow < tElbow) currElbow++; else if (currElbow > tElbow) currElbow--;
    if (currGripper < tGripper) currGripper++; else if (currGripper > tGripper) currGripper--;

    baseServo.write(currBase);
    shoulderServo.write(currShoulder);
    elbowServo.write(currElbow);
    gripperServo.write(currGripper);
    delay(stepDelay);
  }
}
