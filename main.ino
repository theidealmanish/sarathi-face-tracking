#include <AccelStepper.h>

#define DIR_PIN_LEFT 9
#define STEP_PIN_LEFT 10
#define DIR_PIN_RIGHT 4
#define STEP_PIN_RIGHT 5
#define ENABLE_PIN_LEFT 11
#define ENABLE_PIN_RIGHT 6
#define LIMIT_SWITCH_UP 7
#define LIMIT_SWITCH_DOWN 8
#define LIMIT_SWITCH_LEFT 3
#define LIMIT_SWITCH_RIGHT 2
#define AUTO_INPUT_PIN 12
#define MOTOR_INTERFACE_TYPE 1

AccelStepper leftMotor(MOTOR_INTERFACE_TYPE, STEP_PIN_LEFT, DIR_PIN_LEFT);
AccelStepper rightMotor(MOTOR_INTERFACE_TYPE, STEP_PIN_RIGHT, DIR_PIN_RIGHT);

const int MAX_SPEED = 4000;
const int ACCELERATION = 2000;
const int MOVE_SPEED = 3000;
const int HOMING_SPEED = 1200;
const unsigned long IDLE_TIMEOUT = 5000; // 5 seconds

enum State { HOMING, IDLE, GESTURING, MOVING_TO_CENTER };

State currentState = HOMING;

unsigned long lastSerialOutput = 0;
const unsigned long SERIAL_INTERVAL = 800; // Limit serial output to every 500ms

long totalVerticalSteps = 0;
long totalHorizontalSteps = 0;
long centerVerticalPosition = 0;
long centerHorizontalPosition = 0;
long bottomPosition = 0;
long topPosition = 0;
long leftPosition = 0;
long rightPosition = 0;
unsigned long lastMovementTime = 0;
unsigned long gestureStartTime = 0;

void setup() {
  leftMotor.setMaxSpeed(MAX_SPEED);
  leftMotor.setAcceleration(ACCELERATION);
  rightMotor.setMaxSpeed(MAX_SPEED);
  rightMotor.setAcceleration(ACCELERATION);

  pinMode(LIMIT_SWITCH_UP, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_DOWN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);
  pinMode(ENABLE_PIN_LEFT, OUTPUT);
  pinMode(ENABLE_PIN_RIGHT, OUTPUT);
  pinMode(AUTO_INPUT_PIN, INPUT);

  // Initially disable both motors
  digitalWrite(ENABLE_PIN_LEFT, HIGH);
  digitalWrite(ENABLE_PIN_RIGHT, HIGH);

  Serial.begin(9600);
  Serial.println("Vertical and Horizontal Motor Movement Starting...");
}

void loop() {
 
  int upSwitch = digitalRead(LIMIT_SWITCH_UP);
  int downSwitch = digitalRead(LIMIT_SWITCH_DOWN);
  int leftSwitch = digitalRead(LIMIT_SWITCH_LEFT);
  int rightSwitch = digitalRead(LIMIT_SWITCH_RIGHT);

  printStatus(upSwitch, downSwitch, leftSwitch, rightSwitch);

  switch (currentState) {
    case HOMING:
      if (performHoming(upSwitch, downSwitch, leftSwitch, rightSwitch)) {
        currentState = IDLE;
      }
      break;
    case IDLE:
      processSerialInput();
      break;
    case GESTURING:
      break;
    case MOVING_TO_CENTER:
      if (moveToCenter()) {
        currentState = IDLE;
        Serial.println("Moved to center. Entering IDLE state.");
      }
      break;
  }

  leftMotor.run();
  rightMotor.run();
}

void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input from Serial Monitor
    input.trim(); // Remove extra spaces or newline characters

    int commaIndex = input.indexOf(','); // Find the comma separator
    if (commaIndex == -1) {
      Serial.println("Invalid input. Use format: X,Y");
      return;
    }

    // Extract X and Y values from the input
    long targetX = input.substring(0, commaIndex).toInt();
    long targetY = input.substring(commaIndex + 1).toInt();

    Serial.print("Inputs: X=");
    Serial.print(targetX);
    Serial.print(", Y=");
    Serial.println(targetY);

    // Validate the input within the 640x480 range
    if (targetX < 0 || targetX > 640 || targetY < 0 || targetY > 480) {
      Serial.println("Coordinates out of range. Valid range: X=0-640, Y=0-480");
      return;
    }

    // Map screen coordinates to motor steps
    long scaledX = map(targetX, 0, 640, totalHorizontalSteps, 0);
    long scaledY = map(targetY, 0, 480, 0, totalVerticalSteps); // Invert Y-axis for motor steps

    Serial.print("Mapped Inputs to Motor Steps: X=");
    Serial.print(scaledX);
    Serial.print(", Y=");
    Serial.println(scaledY);

    // Calculate offsets from the center position
    long horizontalOffset = scaledX - centerHorizontalPosition;
    long verticalOffset = centerVerticalPosition - scaledY;

    // Set motor targets
    leftMotor.moveTo(centerHorizontalPosition + horizontalOffset + verticalOffset);
    rightMotor.moveTo(centerHorizontalPosition + horizontalOffset - verticalOffset);

    // Print motor target positions
    Serial.print("Moving to: X=");
    Serial.print(scaledX);
    Serial.print(", Y=");
    Serial.println(scaledY);

    Serial.print("Left motor target: ");
    Serial.println(leftMotor.targetPosition());
    Serial.print("Right motor target: ");
    Serial.println(rightMotor.targetPosition());
  }
}

bool performHoming(int upSwitch, int downSwitch, int leftSwitch, int rightSwitch) {
  static int homingStep = 0;
  bool homingComplete = false;

  switch (homingStep) {
    case 0: // Move down
      if (performHomingUp(upSwitch)) homingStep++;
      break;
    case 1: // Move up
      if (performHomingDown(downSwitch)) homingStep++;
      break;
    case 2: // Move to vertical center
      if (moveToVerticalCenter()) homingStep++;
      break;
    case 3: // Move left
      if (performHomingLeft(leftSwitch)) homingStep++;
      break;
    case 4: // Move right
      if (performHomingRight(rightSwitch)) homingStep++;
      break;
    case 5: // Move to center
      if (moveToHorizontalCenter()) {
        homingStep = 0;
        homingComplete = true;
        Serial.println("Homing complete. Waiting for interaction.");
      }
      break;
  }
  return homingComplete;
}

bool moveToVerticalCenter() {
  long verticalSteps = centerVerticalPosition - ((leftMotor.currentPosition() - rightMotor.currentPosition()) / 2);
  
  leftMotor.move(verticalSteps);
  rightMotor.move(-verticalSteps);
  
  leftMotor.run();
  rightMotor.run();
  
  if (leftMotor.distanceToGo() == 0 && rightMotor.distanceToGo() == 0) {
    Serial.println("Reached vertical center.");
    return true;
  }
  
  return false;
}

bool moveToHorizontalCenter() {
  // Calculate the horizontal steps needed for the center
  long horizontalSteps = centerHorizontalPosition - ((leftMotor.currentPosition() + rightMotor.currentPosition()) / 2);

  // Move motors to the calculated horizontal center position
  leftMotor.moveTo(leftMotor.currentPosition() + horizontalSteps / 2);
  rightMotor.moveTo(rightMotor.currentPosition() + horizontalSteps / 2);

  // Run motors and check if they reached their targets
  leftMotor.run();
  rightMotor.run();

  if (leftMotor.distanceToGo() == 0 && rightMotor.distanceToGo() == 0) {
    Serial.println("Reached horizontal center position.");
    return true;
  }

  return false;
}

bool performHomingUp(int upSwitch) {
  if (upSwitch == HIGH) {  // Assuming active-low switches
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
    topPosition = 0;
    Serial.println("Upper limit reached.");
    return true;
  }

  leftMotor.setSpeed(-HOMING_SPEED);
  rightMotor.setSpeed(HOMING_SPEED);
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  return false;
}

bool performHomingDown(int downSwitch) {
  static bool homingDownComplete = false; // Flag to track homing completion

  if (downSwitch == HIGH && !homingDownComplete) {
    // Debounce the switch
    delay(50);
    if (digitalRead(LIMIT_SWITCH_DOWN) == HIGH) { // Confirm the switch is still triggered
      homingDownComplete = true;
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      bottomPosition = leftMotor.currentPosition();  // Record the bottom position
      totalVerticalSteps = abs(topPosition - bottomPosition);  // Calculate total steps
      centerVerticalPosition = totalVerticalSteps / 2;  // Calculate center position
      Serial.print("Homing DOWN: Lower limit reached. Total Vertical Steps: ");
      Serial.print(totalVerticalSteps);
      Serial.print(", Center Position: ");
      Serial.println(centerVerticalPosition);
      return true;
    }
  }

  if (downSwitch == LOW) {
    homingDownComplete = false; // Reset flag when switch is released
  }

  // Move motors downward
  leftMotor.setSpeed(HOMING_SPEED);
  rightMotor.setSpeed(-HOMING_SPEED);
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  return false;
}

bool performHomingLeft(int leftSwitch) {
  if (leftSwitch == HIGH) {  // Assuming active-low switches
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
    leftPosition = 0;
    Serial.println("Left limit reached.");
    return true;
  }

  leftMotor.setSpeed(-HOMING_SPEED);
  rightMotor.setSpeed(-HOMING_SPEED);
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  return false;
}

bool performHomingRight(int rightSwitch) {
  if (rightSwitch == HIGH) {  // Assuming active-low switches
    rightPosition = leftMotor.currentPosition();
    totalHorizontalSteps = abs(rightPosition - leftPosition);
    centerHorizontalPosition = totalHorizontalSteps / 2;
    Serial.print("Horizontal homing complete. Total steps: ");
    Serial.print(totalHorizontalSteps);
    Serial.print(", Center position: ");
    Serial.println(centerHorizontalPosition);
    return true;
  }

  leftMotor.setSpeed(HOMING_SPEED);
  rightMotor.setSpeed(HOMING_SPEED);
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  return false;
}

bool moveToCenter() {
  long horizontalSteps = centerHorizontalPosition - ((leftMotor.currentPosition() + rightMotor.currentPosition()) / 2);
  long verticalSteps = centerVerticalPosition - ((leftMotor.currentPosition() - rightMotor.currentPosition()) / 2);
  
  leftMotor.moveTo(centerHorizontalPosition + centerVerticalPosition);
  rightMotor.moveTo(centerHorizontalPosition - centerVerticalPosition);
  
  if (leftMotor.distanceToGo() == 0 && rightMotor.distanceToGo() == 0) {
    Serial.println("Reached center position.");
    return true;
  }

  return false;
}

void enableMotors() {
  digitalWrite(ENABLE_PIN_LEFT, HIGH);  // Assuming active-low enable
  digitalWrite(ENABLE_PIN_RIGHT, HIGH);
  lastMovementTime = millis();
}

void disableMotors() {
  digitalWrite(ENABLE_PIN_LEFT, LOW);  // Assuming active-low enable
  digitalWrite(ENABLE_PIN_RIGHT, LOW);
}

void checkIdleTimeout() {
  if (millis() - lastMovementTime > IDLE_TIMEOUT) {
    disableMotors();
  }
}

String getStateString(State state) {
  switch (state) {
    case HOMING: return "HOMING";
    case IDLE: return "IDLE";
    case GESTURING: return "GESTURING";
    case MOVING_TO_CENTER: return "MOVING_TO_CENTER";
    default: return "UNKNOWN";
  }
}

void printStatus(int upSwitch, int downSwitch, int leftSwitch, int rightSwitch) {
  if (millis() - lastSerialOutput >= SERIAL_INTERVAL) {
    Serial.print("Up: ");
    Serial.print(upSwitch == HIGH ? "REACHED" : "Not reached");
    Serial.print(" | Down: ");
    Serial.print(downSwitch == HIGH ? "REACHED" : "Not reached");
    Serial.print(" | Left: ");
    Serial.print(leftSwitch == HIGH ? "REACHED" : "Not reached");
    Serial.print(" | Right: ");
    Serial.print(rightSwitch == HIGH ? "REACHED" : "Not reached");
    Serial.print(" | State: ");
    Serial.print(getStateString(currentState));
    Serial.print(" | V-Pos: ");
    Serial.print(leftMotor.currentPosition());
    Serial.print(" | H-Pos: ");
    Serial.print((leftMotor.currentPosition() + rightMotor.currentPosition()) / 2);
    Serial.print(" | Motors: ");
    Serial.println(digitalRead(ENABLE_PIN_LEFT) == HIGH ? "ENABLED" : "DISABLED");
    lastSerialOutput = millis();
  }
}
