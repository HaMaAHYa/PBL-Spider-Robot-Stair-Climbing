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

*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 650

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

int robotDelayMs      = 100;
int robotLegLiftDeg   = 100;
int robotLegPlaceDeg  = 180;

int RF_LB_standby     = 45;
int RF_LB_center      = 0;
int LF_RB_standby     = 135;
int LF_RB_center      = 180;

const int hipLF = 0, legLF = 1;
const int hipRF = 2, legRF = 3;
const int hipLB = 4, legLB = 5;
const int hipRB = 6, legRB = 7;

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.reset();
  pca.setPWMFreq(60);

  standBy();
  delay(4000);
}

void loop() {
  stepLeg(legLF, hipLF, 100);
  swingHips(
    hipLF, LF_RB_center,
    hipLB, RF_LB_standby + 20
  );

  stepLeg(legRB, hipRB, LF_RB_center);
  stepLeg(legRF, hipRF, 80);

  swingHips(
    hipRF, RF_LB_center,
    hipRB, LF_RB_standby - 15
  );

  stepLeg(legLB, hipLB, RF_LB_center);
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

void standBy() {
  setServo(hipRF, RF_LB_standby);
  setServo(hipLB, RF_LB_standby + 20);
  setServo(hipRB, LF_RB_standby - 15);
  setServo(hipLF, LF_RB_standby);

  setServo(legLF, robotLegPlaceDeg);
  setServo(legRF, robotLegPlaceDeg);
  setServo(legLB, robotLegPlaceDeg);
  setServo(legRB, robotLegPlaceDeg);
}

void setServo(int channel, int angle) {
  pca.setPWM(channel, 0, angleToPulse(angle));
}

int angleToPulse(float ang) {
  return map((int)ang, 0, 180, SERVOMIN, SERVOMAX);
}
