/**
 * HKUST ELEC1100 Mini contest
 * Group 97
 * Thomas Ip & Lai Siu Hong
 */

#define SENSOR_L2 A2
#define SENSOR_L1 A3
#define SENSOR_R1 A4
#define SENSOR_R2 A5

#define DIR_L 11
#define DIR_R 12

#define RQ3 3
#define RQ2 4
#define RQ1 5
#define RQ0 6
#define LQ3 7
#define LQ2 8
#define LQ1 9
#define LQ0 10

#define LEFT_90    91
#define RIGHT_90   92
#define Y_JUNCTION 101
#define T_JUNCTION 102

int sensorL2, sensorL1, sensorR1, sensorR2;

/**
 * PID gain parameters
 * Tuning method based on https://robotics.stackexchange.com/a/174
 */
const float Kp = 8;
const float Kd = 6;

int error;
int previousError = 0;
float derivative;
float direction;

// Branch handling
enum Turn {Left, Right};
Turn branchSequence[] = {Left, Left, Right, Left, Right, Right};
int branchIdx = 0;

// Run handling
enum Stage {Waiting, Running1, UTurn, Running2};
Stage stage = Waiting;

boolean enableButton = true;
unsigned long buttonCooldown;

boolean useTruthTable = false;
boolean enable90Turn = false;

/**
 * We implements a PID controller as a control loop feedback mechanism for the
 * robot car.
 * The case of a T junction, Y junction or off track is not handled here.
 */
float pidController(float error)
{
  derivative = error - previousError;
  float output = (Kp * error) + (Kd * derivative);
  return output;
}

/**
 * Turn the robot by decreasing the speed of one of the wheels and leave the
 * other one at maximum speed.
 * +ve direction -> turn right
 * -ve direction -> turn left
 */
void speedControl(int direction)
{
  int leftSpeed = min(15 + direction, 15);
  int rightSpeed = min(15 - direction, 15);
  constantSpeed(leftSpeed, rightSpeed);
}

void constantSpeed(int leftSpeed, int rightSpeed)
{
  digitalWrite(DIR_L, leftSpeed < 0 ? LOW : HIGH);
  digitalWrite(DIR_R, rightSpeed < 0 ? LOW : HIGH);
  if (rightSpeed < 0) {
    rightSpeed -= 1;
  }
  leftSpeed = constrain(abs(leftSpeed), 0, 15);
  rightSpeed = constrain(abs(rightSpeed), 0, 15);

  digitalWrite(LQ0, intToBit(leftSpeed, 0));
  digitalWrite(LQ1, intToBit(leftSpeed, 1));
  digitalWrite(LQ2, intToBit(leftSpeed, 2));
  digitalWrite(LQ3, intToBit(leftSpeed, 3));

  digitalWrite(RQ0, intToBit(rightSpeed, 0));
  digitalWrite(RQ1, intToBit(rightSpeed, 1));
  digitalWrite(RQ2, intToBit(rightSpeed, 2));
  digitalWrite(RQ3, intToBit(rightSpeed, 3));
}

void turnToSetPoint(int leftSpeed, int rightSpeed)
{
  do {
    readSensorValues();
    constantSpeed(leftSpeed, rightSpeed);
  } while (!(sensorL2 && !sensorL1 && !sensorR1 && sensorR2));
}

boolean bumperTriggered() {
  if ((!sensorL2 && !sensorL1 && !sensorR1 && !sensorR2) && (millis() > buttonCooldown) && enableButton) {
    buttonCooldown = millis() + 5000;
    return true;
  }
  return false;
}

/**
 * Return the binary value of an integer at a specified bit.
 */
boolean intToBit(int integer, int bit_)
{
  return (integer >> bit_) & 1;
}

void readSensorValues()
{
  sensorL2 = digitalRead(SENSOR_L2);
  sensorL1 = digitalRead(SENSOR_L1);
  sensorR1 = digitalRead(SENSOR_R1);
  sensorR2 = digitalRead(SENSOR_R2);
}

void setup()
{
  pinMode(SENSOR_L2, INPUT);
  pinMode(SENSOR_L1, INPUT);
  pinMode(SENSOR_R1, INPUT);
  pinMode(SENSOR_R2, INPUT);

  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(LQ3, OUTPUT);
  pinMode(LQ2, OUTPUT);
  pinMode(LQ1, OUTPUT);
  pinMode(LQ0, OUTPUT);
  pinMode(RQ3, OUTPUT);
  pinMode(RQ2, OUTPUT);
  pinMode(RQ1, OUTPUT);
  pinMode(RQ0, OUTPUT);

  constantSpeed(0, 0);

  buttonCooldown = millis();
  previousMicros = micros();
}

void loop()
{
  readSensorValues();

  switch (stage) {
  case Waiting:
    if (bumperTriggered()) {
      do {
        readSensorValues();
        delay(100);
      } while (!sensorL2 && !sensorL1 && !sensorR1 && !sensorR2);
      stage = Running1;
      enableButton = false;
    }
    return;
  case Running1:
    if (bumperTriggered()) {
      stage = UTurn;
      enableButton = false;
      return;
    }
    break;
  case UTurn:
    constantSpeed(-15, -15);
    delay(100);
    constantSpeed(-15, 15);
    delay(100);
    turnToSetPoint(-15, 15);
    stage = Running2;
    return;
  case Running2:
    if (bumperTriggered()) {
      constantSpeed(0, 0);
      delay(100000);
    }
    break;
  }

  if      ((!sensorL2 && !sensorR2) && !enable90Turn) error = T_JUNCTION;  // Try !L2 !L1 R1 !R2 without enable90Turn
  else if ( sensorL2 &&  sensorL1 &&  sensorR1 && !sensorR2) error = 3;
  else if ( sensorL2 &&  sensorL1 && !sensorR1 && !sensorR2) error = 2;
  else if ( sensorL2 &&  sensorL1 && !sensorR1 &&  sensorR2) error = 1;
  else if ( sensorL2 && !sensorL1 && !sensorR1 &&  sensorR2) error = 0;
  else if ( sensorL2 && !sensorL1 &&  sensorR1 &&  sensorR2) error = -1;
  else if (!sensorL2 && !sensorL1 &&  sensorR1 &&  sensorR2) error = -2;
  else if (!sensorL2 &&  sensorL1 &&  sensorR1 &&  sensorR2) error = -3;

  switch (error) {
  case T_JUNCTION:
    switch (branchSequence[branchIdx++]) {
    case Left: turnToSetPoint(-7, 15); break;
    case Right: turnToSetPoint(15, -7); break;
    }

    if (branchIdx == 2) useTruthTable = true;
    else if (branchIdx == 3) {
      enableButton = enable90Turn = true;
      useTruthTable = false;
    }
    else if (branchIdx == 4) enable90Turn = false;  // unreachable due to T condition
    else if (branchIdx == 6) enableButton = true;
    break;
  default:
    if (enable90Turn) {
      if (!sensorL2 && sensorR2) {
        constantSpeed(-8, 13);
      } else if (sensorL2 && !sensorR2) {
        constantSpeed(13, -8);
      } else {
        constantSpeed(15, 15);
      }
    } else {
      direction = pidController(error);
      speedControl((int) direction);
      previousError = error;
    }
  }
}
