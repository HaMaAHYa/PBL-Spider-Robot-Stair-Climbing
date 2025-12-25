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


            12 echo13 ultrasonic
            8 to driver servo for sensor                 

*/

String command = "";
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
int step = 5;             // +1 or -1
unsigned long lastMove = 0;
unsigned long scanStepTime = 20; // ms

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.reset();
  pca.setPWMFreq(60);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  standBy();
  delay(4000);
}

void loop() {
  /*
   unsigned long now = millis();

  // Move servo step-by-step
  if (now - lastMove >= scanStepTime) {
    lastMove = now;

    currentAngle += step;

    // Reverse direction at limits
    if (currentAngle >= leftAngle || currentAngle <= rightAngle) {
      step = -step;
    }

    pca.setPWM(sensorServo, 0, angleToPulse(currentAngle));

    // Read sensor at SAME TIME
    int distance = getDistance();

    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print("  Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  */
  
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "l") {
        Serial.println("Left");
        for (int i = 0; i < 2; i++) {
          moveLeft();
        }
        safeLeft();
      } else if (command == "r") {
        Serial.println("Right");
        for (int i = 0; i < 2; i++) {
          moveRight();
        }
        safeRight();
      } else if (command == "s") {
        standBy();
        Serial.println("stand");
      }
  }

}




void safeLeft() {
  safePose(LeftMap);
}

void safeRight() {
  safePose(RightMap);
}

void safePose(const ServoMap& map) {
  setServo(map.hipRF, 90);
  setServo(map.hipLB, RF_LB_standby + 20);
  setServo(map.hipRB, LF_RB_standby - 15);
  setServo(map.hipLF, 90);

  setServo(map.legLF, 120);
  setServo(map.legRF, 120);
  setServo(map.legLB, robotLegPlaceDeg);
  setServo(map.legRB, robotLegPlaceDeg);
}


void moveLeft() {
  // Step 1: LF
  stepLeg(LeftMap.legLF, LeftMap.hipLF, 100);

  // Step 2: swing body
  swingHips(
    LeftMap.hipLF, LF_RB_center,
    LeftMap.hipLB, RF_LB_standby + 20
  );

  // Step 3: RB
  stepLeg(LeftMap.legRB, LeftMap.hipRB, LF_RB_center);

  // Step 4: RF
  stepLeg(LeftMap.legRF, LeftMap.hipRF, 80);

  // Step 5: swing body
  swingHips(
    LeftMap.hipRF, RF_LB_center,
    LeftMap.hipRB, LF_RB_standby - 15
  );

  // Step 6: LB
  stepLeg(LeftMap.legLB, LeftMap.hipLB, RF_LB_center);
}


void moveRight() {
  stepLeg(RightMap.legLF, RightMap.hipLF, 100);

  swingHips(
    RightMap.hipLF, LF_RB_center,
    RightMap.hipLB, RF_LB_standby + 20
  );

  stepLeg(RightMap.legRB, RightMap.hipRB, LF_RB_center);

  stepLeg(RightMap.legRF, RightMap.hipRF, 80);

  swingHips(
    RightMap.hipRF, RF_LB_center,
    RightMap.hipRB, LF_RB_standby - 15
  );

  stepLeg(RightMap.legLB, RightMap.hipLB, RF_LB_center);
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

void stepLeg(int leg, int hip, int hipAngle) {
  setServo(leg, robotLegLiftDeg);
  delay(robotDelayMs);

  setServo(hip, hipAngle);
  delay(robotDelayMs);

  setServo(leg, robotLegPlaceDeg);
  delay(robotDelayMs + 50);
}

void swingHips(int hip1, int ang1, int hip2, int ang2) {
  setServo(hip1, ang1);
  setServo(hip2, ang2);
  delay(robotDelayMs + 100);
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

void TEST_MOVE_LEFT_RIGHT() {
  for (int i  = 0; i <= 7; i++) {
    if (i < 4) {
      moveLeft();
    } else if ( i == 4) {
      standBy();
      delay(500);
    } else {
      moveRight();
    }
  }
  standBy();
  delay(500);
}
