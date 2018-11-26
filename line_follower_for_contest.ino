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

#define Y_JUNCTION 101
#define T_JUNCTION 102

int sensorL2, sensorL1, sensorR1, sensorR2;

/**
 * PID gain parameters
 * Tuning method based on https://robotics.stackexchange.com/a/174
 */
const float Kp = 8;
const float Ki = 0;
const float Kd = 5;

int error;
int previousError = 0;
float integral = 0;
float derivative;
float direction;

int loopDuration;

// Branch handling
enum Turn {Left, Right};
Turn branchSequence[] = {Left, Left, Right, Left, Right, Right};
int branchIdx = 0;

// Run handling
enum Stage {Waiting, Running1, UTurn, Running2, Finish};
Stage stage = Waiting;

/**
 * We implements a PID controller as a control loop feedback mechanism for the
 * robot car.
 * The case of a T junction, Y junction or off track is not handled here.
 */
float pidController(float error)
{
  float dt = loopDuration / 1000;
  integral += error * dt;
  derivative = (error - previousError) / dt;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
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

  previousMillis = millis();
}

void loop()
{
  readSensorValues();

  switch (stage) {
  case Waiting:
    if (!sensorL2 && !sensorL1 && !sensorR1 && !sensorR2) stage = Running1;
    return;
  case Running1:
    if (!sensorL2 && sensorL1 && !sensorR1 && !sensorR2) stage = UTurn;
    break;
  case UTurn:
    turnToSetPoint(15, -15);
    stage = Running2;
    return;
  case Running2:
    if (!sensorL2 && sensorL1 && !sensorR1 && !sensorR2) stage = Finish;
    break;
  case Finish:
    constantSpeed(0, 0);
    delay(100000);
  }

  if      (!sensorL2 &&  sensorL1 &&  sensorR1 && !sensorR2) error = Y_JUNCTION;
  else if (!sensorL2 && !sensorL1 && !sensorR1 && !sensorR2) error = T_JUNCTION;
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
    case Left:
      turnToSetPoint(-7, 12);
      break;
    case Right:
      turnToSetPoint(12, -7);
      break;
    }
    break;
  case Y_JUNCTION:
    break;
  default:
    direction = pidController(error);
    speedControl((int) direction);
    previousError = error;
  }

  unsigned long currentMillis = millis();
  loopDuration = currentMillis - previousError;
}
