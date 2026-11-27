#include <Servo.h>

// ================= ARM PWM PINS (ARDUINO MEGA) =================
const int BASE_PIN     = 6;  // Rotates arm from front shelf (0°) to rear bin (180°)
const int SHOULDER_PIN = 7;  // Lifts and lowers main arm boom
const int ELBOW_PIN    = 8;  // Extends forward toward shelf / folds back
const int GRIPPER_PIN  = 9;  // Opens and closes claw

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

// Current joint angles (starting at safe stowed home position)
int currBase     = 90;
int currShoulder = 110;
int currElbow    = 120;
int currGripper  = 40;  // Open

// ================= CALIBRATED POSE ANGLES (TUNE THESE) =================
// 1. Stowed Home Position
const int HOME_BASE = 90, HOME_SHOULDER = 110, HOME_ELBOW = 120, GRIP_OPEN = 40;

// 2. Front Shelf Pickup Position
const int PICK_BASE = 90, PICK_SHOULDER = 65,  PICK_ELBOW = 70,  GRIP_CLOSE = 115;

// 3. Rear Self-Contained Bin Drop Position (180° rotated to back)
const int DROP_BASE = 180, DROP_SHOULDER = 85, DROP_ELBOW = 95;

void setup() {
  Serial.begin(115200);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // Move slowly to initial home pose
  moveSmooth(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_OPEN, 15);

  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore newline/carriage return characters
    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      case 't': // Run full Pick -> Rotate to Rear Bin -> Drop -> Home test cycle
      case 'T':
        runPickAndPlaceCycle();
        break;

      case 'h': // Return to stowed Home pose
      case 'H':
        Serial.println("Moving to HOME pose...");
        moveSmooth(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, currGripper, 15);
        break;

      // --- Manual Joint Stepping (+5 / -5 degrees) for Calibration ---
      case 'b': currBase = constrain(currBase - 5, 0, 180); baseServo.write(currBase); break;
      case 'B': currBase = constrain(currBase + 5, 0, 180); baseServo.write(currBase); break;

      case 's': currShoulder = constrain(currShoulder - 5, 0, 180); shoulderServo.write(currShoulder); break;
      case 'S': currShoulder = constrain(currShoulder + 5, 0, 180); shoulderServo.write(currShoulder); break;

      case 'e': currElbow = constrain(currElbow - 5, 0, 180); elbowServo.write(currElbow); break;
      case 'E': currElbow = constrain(currElbow + 5, 0, 180); elbowServo.write(currElbow); break;

      case 'g': currGripper = constrain(currGripper - 5, 0, 180); gripperServo.write(currGripper); break;
      case 'G': currGripper = constrain(currGripper + 5, 0, 180); gripperServo.write(currGripper); break;

      default:
        printMenu();
        break;
    }

    printCurrentAngles();
  }
}

// ================= SMOOTH MOTION HELPER =================
// Moves all 4 joints simultaneously at a controlled speed to prevent jerking
void moveSmooth(int targetBase, int targetShoulder, int targetElbow, int targetGripper, int stepDelayMs) {
  while (currBase != targetBase || currShoulder != targetShoulder || 
         currElbow != targetElbow || currGripper != targetGripper) {
    
    if (currBase < targetBase) currBase++;
    else if (currBase > targetBase) currBase--;

    if (currShoulder < targetShoulder) currShoulder++;
    else if (currShoulder > targetShoulder) currShoulder--;

    if (currElbow < targetElbow) currElbow++;
    else if (currElbow > targetElbow) currElbow--;

    if (currGripper < targetGripper) currGripper++;
    else if (currGripper > targetGripper) currGripper--;

    baseServo.write(currBase);
    shoulderServo.write(currShoulder);
    elbowServo.write(currElbow);
    gripperServo.write(currGripper);

    delay(stepDelayMs);
  }
}

// ================= FULL PICK & REAR BIN PLACE SEQUENCE =================
void runPickAndPlaceCycle() {
  Serial.println("\n--- Starting Pick & Rear Bin Stowage Test ---");

  // 1. Open gripper and reach toward front shelf item
  Serial.println("1. Reaching for shelf item...");
  moveSmooth(PICK_BASE, PICK_SHOULDER, PICK_ELBOW, GRIP_OPEN, 15);
  delay(400);

  // 2. Close gripper to grasp item
  Serial.println("2. Grasping item...");
  moveSmooth(PICK_BASE, PICK_SHOULDER, PICK_ELBOW, GRIP_CLOSE, 15);
  delay(500);

  // 3. Lift item clear of shelf before rotating
  Serial.println("3. Lifting item clear...");
  moveSmooth(PICK_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_CLOSE, 15);
  delay(300);

  // 4. Rotate base 180° to face the rear self-contained container
  Serial.println("4. Rotating to rear container...");
  moveSmooth(DROP_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_CLOSE, 15);
  delay(300);

  // 5. Lower slightly over rear container and release item
  Serial.println("5. Dropping item into rear container...");
  moveSmooth(DROP_BASE, DROP_SHOULDER, DROP_ELBOW, GRIP_CLOSE, 15);
  delay(300);
  moveSmooth(DROP_BASE, DROP_SHOULDER, DROP_ELBOW, GRIP_OPEN, 15);
  delay(500);

  // 6. Lift arm and return to stowed home pose
  Serial.println("6. Returning to stowed HOME position...");
  moveSmooth(DROP_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_OPEN, 15);
  moveSmooth(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, GRIP_OPEN, 15);

  Serial.println("--- Test Cycle Complete ---\n");
}

void printCurrentAngles() {
  Serial.print("Angles -> Base: "); Serial.print(currBase);
  Serial.print(" | Shoulder: ");     Serial.print(currShoulder);
  Serial.print(" | Elbow: ");        Serial.print(currElbow);
  Serial.print(" | Gripper: ");      Serial.println(currGripper);
}

void printMenu() {
  Serial.println("\n=== Robotic Arm Calibration & Test Menu ===");
  Serial.println("  [T] Run full Pick -> Rear Bin Drop -> Home sequence");
  Serial.println("  [H] Move to Home pose");
  Serial.println("  [b / B] Base -5° / +5°      [s / S] Shoulder -5° / +5°");
  Serial.println("  [e / E] Elbow -5° / +5°     [g / G] Gripper -5° / +5°\n");
}
