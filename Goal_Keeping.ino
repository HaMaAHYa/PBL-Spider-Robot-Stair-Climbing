/*

         ROBOT CONFIGULATION                                  

            90        90                                              
            |    ^    |                                               
    180_____|____|____|_____0                                       
            |0       2|                                       
            |         |                                 
      0_____|4_______6|_____180                 
            |         |                       
            |         |                         
            90        90         
           
to do:
  CHANGE THE MOVEMENT AND MAKE IT MOVE FASTER
  

*/

String command = "";

#define MAX_ANGLE 180
int distanceMap[MAX_ANGLE + 1];

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 650

#define TRIG_PIN 13
#define ECHO_PIN 12
const int sensorServo = 8;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

enum RobotState {
  STOP,
  WALK_FORWARD,
  WALK_BACKWARD
};

RobotState robotState = STOP;

struct ServoMap {
  int hipLF;
  int hipRF;
  int hipLB;
  int hipRB;
  int legLF;
  int legRF;
  int legLB;
  int legRB;
};

ServoMap LeftMap = {
  .hipLF = 0,
  .hipRF = 2,
  .hipLB = 4,
  .hipRB = 6,
  .legLF = 1,
  .legRF = 3,
  .legLB = 5,
  .legRB = 7
};

ServoMap RightMap = {
  .hipLF = 6,
  .hipRF = 4,
  .hipLB = 2,
  .hipRB = 0,
  .legLF = 7,
  .legRF = 5,
  .legLB = 3,
  .legRB = 1
};

int robotDelayMs      = 100;
int robotLegLiftDeg   = 100;
int robotLegPlaceDeg  = 180;

int RF_LB_standby     = 45;
int RF_LB_center      = 0;
int LF_RB_standby     = 135;
int LF_RB_center      = 180;

int rightAngle  = 0;
int leftAngle   = 90;

int currentAngle = rightAngle;
int step = 5;
unsigned long lastMove = 0;
unsigned long scanStepTime = 10;

int nearestDistance = 999;
int nearestAngle = 0;

unsigned long scanStartTime = 0;
unsigned long scanDuration = 1500;

// Movement state machine
enum MovementState {
  IDLE,
  MOVE_LEFT_STEP1,
  MOVE_LEFT_STEP1_WAIT,
  MOVE_LEFT_STEP2,
  MOVE_LEFT_STEP2_WAIT,
  MOVE_LEFT_STEP3,
  MOVE_LEFT_STEP3_WAIT,
  MOVE_LEFT_STEP4,
  MOVE_LEFT_STEP4_WAIT,
  MOVE_LEFT_STEP5,
  MOVE_LEFT_STEP5_WAIT,
  MOVE_LEFT_STEP6,
  MOVE_LEFT_STEP6_WAIT,
  MOVE_RIGHT_STEP1,
  MOVE_RIGHT_STEP1_WAIT,
  MOVE_RIGHT_STEP2,
  MOVE_RIGHT_STEP2_WAIT,
  MOVE_RIGHT_STEP3,
  MOVE_RIGHT_STEP3_WAIT,
  MOVE_RIGHT_STEP4,
  MOVE_RIGHT_STEP4_WAIT,
  MOVE_RIGHT_STEP5,
  MOVE_RIGHT_STEP5_WAIT,
  MOVE_RIGHT_STEP6,
  MOVE_RIGHT_STEP6_WAIT
};

MovementState movementState = IDLE;
unsigned long movementTimer = 0;

// Sub-states for stepLeg
enum StepLegSubState {
  STEPLEG_LIFT,
  STEPLEG_MOVE,
  STEPLEG_PLACE,
  STEPLEG_DONE
};

StepLegSubState stepLegSubState = STEPLEG_DONE;
int stepLeg_leg, stepLeg_hip, stepLeg_hipAngle;

// Sub-states for swingHips
enum SwingHipsSubState {
  SWINGHIPS_MOVE,
  SWINGHIPS_DONE
};

SwingHipsSubState swingHipsSubState = SWINGHIPS_DONE;

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.reset();
  pca.setPWMFreq(60);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for (int i = 0; i <= MAX_ANGLE; i++) {
    distanceMap[i] = -1;
  }

  standBy();
  delay(3000);
}

void loop() {
  unsigned long now = millis();

  if (scanStartTime == 0) {
    scanStartTime = now;
  }

  // ================= SCANNING =================
  if (now - lastMove >= scanStepTime) {
    lastMove = now;

    currentAngle += step;

    if (currentAngle >= leftAngle || currentAngle <= rightAngle) {
      step = -step;
    }

    setServo(sensorServo, currentAngle);

    int distance = getDistance();

    if (distance > 0 && distance < nearestDistance) {
      nearestDistance = distance;
      nearestAngle = currentAngle;
    }

    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print("  Distance: ");
    Serial.print(distance);
    Serial.print("  Nearest Distance: ");
    Serial.print(nearestDistance);
    Serial.print("  Nearest Angle: ");
    Serial.println(nearestAngle);

    // Only trigger movement if idle
    if (movementState == IDLE) {
      if (nearestAngle > 60) {
        startMoveLeft();
      } else if (nearestAngle > 40) {
        standBy();
      } else {
        startMoveRight();
      }
    }
  }

  // Process movement state machine
  processMovement(now);

  // Testing via monitor
  movementTestingViaMonitor();
}

void processMovement(unsigned long now) {
  if (movementState == IDLE) return;

  switch (movementState) {
    // ================= MOVE LEFT =================
    case MOVE_LEFT_STEP1:
      stepLegStart(LeftMap.legLF, LeftMap.hipLF, 100);
      movementState = MOVE_LEFT_STEP1_WAIT;
      break;

    case MOVE_LEFT_STEP1_WAIT:
      if (stepLegProcess(now)) {
        movementState = MOVE_LEFT_STEP2;
      }
      break;

    case MOVE_LEFT_STEP2:
      swingHipsStart(LeftMap.hipLF, LF_RB_center, LeftMap.hipLB, RF_LB_standby + 20);
      movementState = MOVE_LEFT_STEP2_WAIT;
      break;

    case MOVE_LEFT_STEP2_WAIT:
      if (swingHipsProcess(now)) {
        movementState = MOVE_LEFT_STEP3;
      }
      break;

    case MOVE_LEFT_STEP3:
      stepLegStart(LeftMap.legRB, LeftMap.hipRB, LF_RB_center);
      movementState = MOVE_LEFT_STEP3_WAIT;
      break;

    case MOVE_LEFT_STEP3_WAIT:
      if (stepLegProcess(now)) {
        movementState = MOVE_LEFT_STEP4;
      }
      break;

    case MOVE_LEFT_STEP4:
      stepLegStart(LeftMap.legRF, LeftMap.hipRF, 80);
      movementState = MOVE_LEFT_STEP4_WAIT;
      break;

    case MOVE_LEFT_STEP4_WAIT:
      if (stepLegProcess(now)) {
        movementState = MOVE_LEFT_STEP5;
      }
      break;

    case MOVE_LEFT_STEP5:
      swingHipsStart(LeftMap.hipRF, RF_LB_center, LeftMap.hipRB, LF_RB_standby - 15);
      movementState = MOVE_LEFT_STEP5_WAIT;
      break;

    case MOVE_LEFT_STEP5_WAIT:
      if (swingHipsProcess(now)) {
        movementState = MOVE_LEFT_STEP6;
      }
      break;

    case MOVE_LEFT_STEP6:
      stepLegStart(LeftMap.legLB, LeftMap.hipLB, RF_LB_center);
      movementState = MOVE_LEFT_STEP6_WAIT;
      break;

    case MOVE_LEFT_STEP6_WAIT:
      if (stepLegProcess(now)) {
        movementState = IDLE;
      }
      break;

    // ================= MOVE RIGHT =================
    case MOVE_RIGHT_STEP1:
      stepLegStart(RightMap.legLF, RightMap.hipLF, 100);
      movementState = MOVE_RIGHT_STEP1_WAIT;
      break;

    case MOVE_RIGHT_STEP1_WAIT:
      if (stepLegProcess(now)) {
        movementState = MOVE_RIGHT_STEP2;
      }
      break;

    case MOVE_RIGHT_STEP2:
      swingHipsStart(RightMap.hipLF, LF_RB_center, RightMap.hipLB, RF_LB_standby + 20);
      movementState = MOVE_RIGHT_STEP2_WAIT;
      break;

    case MOVE_RIGHT_STEP2_WAIT:
      if (swingHipsProcess(now)) {
        movementState = MOVE_RIGHT_STEP3;
      }
      break;

    case MOVE_RIGHT_STEP3:
      stepLegStart(RightMap.legRB, RightMap.hipRB, LF_RB_center);
      movementState = MOVE_RIGHT_STEP3_WAIT;
      break;

    case MOVE_RIGHT_STEP3_WAIT:
      if (stepLegProcess(now)) {
        movementState = MOVE_RIGHT_STEP4;
      }
      break;

    case MOVE_RIGHT_STEP4:
      stepLegStart(RightMap.legRF, RightMap.hipRF, 80);
      movementState = MOVE_RIGHT_STEP4_WAIT;
      break;

    case MOVE_RIGHT_STEP4_WAIT:
      if (stepLegProcess(now)) {
        movementState = MOVE_RIGHT_STEP5;
      }
      break;

    case MOVE_RIGHT_STEP5:
      swingHipsStart(RightMap.hipRF, RF_LB_center, RightMap.hipRB, LF_RB_standby - 15);
      movementState = MOVE_RIGHT_STEP5_WAIT;
      break;

    case MOVE_RIGHT_STEP5_WAIT:
      if (swingHipsProcess(now)) {
        movementState = MOVE_RIGHT_STEP6;
      }
      break;

    case MOVE_RIGHT_STEP6:
      stepLegStart(RightMap.legLB, RightMap.hipLB, RF_LB_center);
      movementState = MOVE_RIGHT_STEP6_WAIT;
      break;

    case MOVE_RIGHT_STEP6_WAIT:
      if (stepLegProcess(now)) {
        movementState = IDLE;
      }
      break;
  }
}

// ================= STEPLEG FUNCTIONS =================
void stepLegStart(int leg, int hip, int hipAngle) {
  stepLeg_leg = leg;
  stepLeg_hip = hip;
  stepLeg_hipAngle = hipAngle;
  stepLegSubState = STEPLEG_LIFT;
  movementTimer = millis();
}

bool stepLegProcess(unsigned long now) {
  if (now - movementTimer < robotDelayMs) return false;

  switch (stepLegSubState) {
    case STEPLEG_LIFT:
      setServo(stepLeg_leg, robotLegLiftDeg);
      stepLegSubState = STEPLEG_MOVE;
      movementTimer = now;
      return false;

    case STEPLEG_MOVE:
      setServo(stepLeg_hip, stepLeg_hipAngle);
      stepLegSubState = STEPLEG_PLACE;
      movementTimer = now;
      return false;

    case STEPLEG_PLACE:
      setServo(stepLeg_leg, robotLegPlaceDeg);
      stepLegSubState = STEPLEG_DONE;
      movementTimer = now;
      return false;

    case STEPLEG_DONE:
      if (now - movementTimer >= 50) {  // Extra 50ms delay
        return true;  // Completed
      }
      return false;
  }
  return false;
}

// ================= SWINGHIPS FUNCTIONS =================
void swingHipsStart(int hip1, int ang1, int hip2, int ang2) {
  setServo(hip1, ang1);
  setServo(hip2, ang2);
  swingHipsSubState = SWINGHIPS_MOVE;
  movementTimer = millis();
}

bool swingHipsProcess(unsigned long now) {
  if (swingHipsSubState == SWINGHIPS_MOVE) {
    if (now - movementTimer >= robotDelayMs + 100) {
      swingHipsSubState = SWINGHIPS_DONE;
      return true;  // Completed
    }
  }
  return false;
}

void startMoveLeft() {
  standBy();
  movementState = MOVE_LEFT_STEP1;
}

void startMoveRight() {
  standBy();
  movementState = MOVE_RIGHT_STEP1;
}

void driveLeft() {
  safePose(LeftMap);
}

void driveRight() {
  safePose(RightMap);
}

void safePose(const ServoMap& map) {
  setServo(map.hipRF, 90);
  setServo(map.hipLB, RF_LB_standby + 20);
  setServo(map.hipRB, LF_RB_standby - 15);
  setServo(map.hipLF, 90);

  setServo(map.legLF, 120);
  setServo(map.legRF, 45);
  setServo(map.legLB, robotLegPlaceDeg);
  setServo(map.legRB, robotLegPlaceDeg);
}

void standBy() {
  setServo(LeftMap.hipRF, RF_LB_standby);
  setServo(LeftMap.hipLB, RF_LB_standby + 20);
  setServo(LeftMap.hipRB, LF_RB_standby - 15);
  setServo(LeftMap.hipLF, LF_RB_standby);

  setServo(LeftMap.legLF, robotLegPlaceDeg);
  setServo(LeftMap.legRF, robotLegPlaceDeg);
  setServo(LeftMap.legLB, robotLegPlaceDeg);
  setServo(LeftMap.legRB, robotLegPlaceDeg);
}

void setServo(int channel, int angle) {
  pca.setPWM(channel, 0, angleToPulse(angle));
}

int angleToPulse(float ang) {
  return map((int)ang, 0, 180, SERVOMIN, SERVOMAX);
}

int getDistance() {
  long duration;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return -1;

  return duration / 58;
}

void movementTestingViaMonitor() {
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "l" && movementState == IDLE) {
      Serial.println("Left");
      startMoveLeft();
    } else if (command == "r" && movementState == IDLE) {
      Serial.println("Right");
      startMoveRight();
    } else if (command == "s") {
      standBy();
      movementState = IDLE;
      Serial.println("stand");
    }
  }
}
