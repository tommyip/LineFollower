/**
 * HKUST ELEC1100 Final project
 * Group 97
 * Thomas Ip & Lai Siu Hong
 */

#define SENSOR_F A2
#define SENSOR_R A3
#define SENSOR_M A4
#define SENSOR_L A5

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

#define OFF_TRACK 100
#define Y_JUNCTION 101
#define T_JUNCTION 102

int sensorLeft, sensorMiddle, sensorRight, sensorFront;

/**
 * PID gain parameters
 * Tuning method based on https://robotics.stackexchange.com/a/174
 */
const float Kp = 14;
const float Ki = 0;
const float Kd = 5;

const int pidFrequency = 1000;
unsigned long targetLoopDuration = (1 / pidFrequency) * 1000;
unsigned long previousMillis, loopDuration;

int error;
int previousError = 0;
float integral = 0;
float derivative;
float direction;

// Branch handling
enum Turn {Left, Right, Neutral};
Turn branchSequence[] = {Left, Left, Right};
int branchIdx = 0;

// Run handling
enum Stage {Waiting, Running, Finish, Idle};
Stage stage = Waiting;

// The PID controller does not work well with 90 degrees turn, so we fall back
// to a truth table based line follower algorithm after the last Y junction.
boolean handleRightAngleTurn = false;

/**
 * We implements a PID controller as a control loop feedback mechanism for the
 * robot car. There are 5 possible error values that can be fed into the
 * controller,
 * -2 : Far left
 * -1 : Slightly left
 *  0 : Middle (set point)
 *  1 : Slightly right
 *  2 : Far right
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
  } while (!(sensorLeft && !sensorMiddle && sensorRight));
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
  sensorLeft = digitalRead(SENSOR_L);
  sensorMiddle = digitalRead(SENSOR_M);
  sensorRight = digitalRead(SENSOR_R);
  sensorFront = digitalRead(SENSOR_F);
}

void setup()
{
  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_M, INPUT);
  pinMode(SENSOR_R, INPUT);
  pinMode(SENSOR_F, INPUT);

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
    if (!sensorFront) stage = Running;
    return;
  case Running:
    if (handleRightAngleTurn && !sensorFront) stage = Finish;
    break;
  case Finish:
    constantSpeed(-15, -15);
    delay(1500);
    stage = Idle;
    break;
  case Idle:
    constantSpeed(0, 0);
    return;
  }

  if      ( sensorLeft &&  sensorMiddle &&  sensorRight) error = OFF_TRACK;
  else if (!sensorLeft &&  sensorMiddle && !sensorRight) error = Y_JUNCTION;
  else if (!sensorLeft && !sensorMiddle && !sensorRight) error = T_JUNCTION;
  else if ( sensorLeft &&  sensorMiddle && !sensorRight) error = 2;
  else if ( sensorLeft && !sensorMiddle && !sensorRight) error = 1;
  else if ( sensorLeft && !sensorMiddle &&  sensorRight) error = 0;
  else if (!sensorLeft && !sensorMiddle &&  sensorRight) error = -1;
  else if (!sensorLeft &&  sensorMiddle &&  sensorRight) error = -2;

  switch (error) {
  case OFF_TRACK:
    break;
  case T_JUNCTION:
    if (!handleRightAngleTurn) {
      switch (branchSequence[branchIdx++]) {
      case Left:
        turnToSetPoint(-7, 12);
        break;
      case Right:
        turnToSetPoint(12, -7);
        // 90-degree turn starts after the last Y junction
        handleRightAngleTurn = true;
        targetLoopDuration = 0;
        break;
      }
    }
    break;
  case Y_JUNCTION:
    break;
  default:
    if (handleRightAngleTurn) {
      switch (error) {
      case 1: constantSpeed(13, -8); return;
      case -1: constantSpeed(-8, 13); return;
      case 0: constantSpeed(12, 12); return;
      case 2: constantSpeed(15, -15); return;
      case -2: constantSpeed(-15, 15); return;
      }
    } else {
      direction = pidController(error);
      speedControl((int) direction);
    }
    previousError = error;
  }

  // Run the main loop at a constant frequency
  unsigned long currentMillis = millis();
  loopDuration = currentMillis - previousError;
  previousMillis = currentMillis;

  int timeLeft = targetLoopDuration - loopDuration;

  delay(max(timeLeft, 0));
}
