#include "Arduino.h"
#include "ESP32Encoder.h" //uses esp32 PCNT (pulse counter) instead of interrupt
#include "PID_v1.h"       //pid controller
#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include "parameters.h"   //unkown map and solve the path

std::vector<coord> gates = {g1, g2, g3, g4};
std::vector<coord> obstacles = {o1, o2, o3, o4, o5, o6, o7, o8, o9, o10};
std::vector<int> path = complete_sequence(start, end, gates, obstacles);
int estimatedTime = count_moves(path)[3];

unsigned long startTime;

// physical parameters
const double diameter = 6.985;      // wheel diameter(cm)
const double ticksPerRev = 11 * 62; // ticks per revolution of motor_output, 11 ticks and gear ratio is 62 experimentally calculated
const double forwardDist = 25;      // 25 cm forward(half of a square), idk why its 12.5

//TODO - FIX THESE DAMN STUPID TIMES
const unsigned long forwardDuration = 1145;
const unsigned long turn90Duration = 805;
const unsigned long turn180Duration = 1540;

// create pid controller for motors(distance)
double motor_Kp = 3; // 2 works
double motor_Ki = 0;
double motor_Kd = 0.1;
double motor_input;
double motor_output;
double motor_setpoint;
PID motorPID(&motor_input, &motor_output, &motor_setpoint, motor_Kp, motor_Ki, motor_Kd, DIRECT);
double forward_threshold = 11; // encoder counts threshold

// create pid controller using gyroscope for curr_heading
double heading_Kp = 0.5; // 0.4 works for straight
double heading_Ki = 0;   // 0.3 works for straight
double heading_Kd = 0;   // DONT ADD KD
double heading_input;
double heading_output;
double heading_setpoint;
PID headingPID(&heading_input, &heading_output, &heading_setpoint, heading_Kp, heading_Ki, heading_Kd, REVERSE);

// turning
double turn_Kp = 3.32; // 3.32 works
double turn_Ki = 0; // 1.0 works but can mess stuff up
double turn_Kd = 0; // DO NOT ADD KD (pos - negative exponentially increase error)
double turn_input;
double turn_output;
double turn_setpoint;
double turn_threshold = 5; // turning exit condition
PID turnPID(&turn_input, &turn_output, &turn_setpoint, turn_Kp, turn_Ki, turn_Kd, REVERSE);

// create encoder objects
ESP32Encoder left;
ESP32Encoder right;

// data pins for encoders
#define leftPinA 15
#define leftPinB 4
#define rightPinA 19
#define rightPinB 18

// motor pins, en pins use pwm for speed, in pins control direction(one high one low)
#define enLeft 26
#define inLeft1 14
#define inLeft2 27

#define enRight 32
#define inRight1 25
#define inRight2 33

// pwm channels(0-15)
#define leftChannel 0
#define rightChannel 1

#define statusLED 2 // onboard led
#define enableButton 13

// mpu6050, 22 is scl, 21 is sda
Adafruit_MPU6050 mpu;
float GyroZ;
float yaw;
float GyroErrorZ;
float elapsedTime;
float currentTime;
float previousTime;

int curr_heading = 0;

void calculateIMUError()
{
  // Read gyro values 200 times
  for (int i = 0; i < 200; i++)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Sum all readings
    GyroErrorZ = GyroErrorZ + g.gyro.z;
  }
  // Divide the sum by 200 to get the error value
  GyroErrorZ = GyroErrorZ / 200;
}

void calculateHeading()
{
  // === Read gyroscope data === //
  previousTime = currentTime;                        // Previous time is stored before the actual time read
  currentTime = millis();                            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Integrate gyro z (radians/second) over seconds to get degrees
  yaw += (g.gyro.z - GyroErrorZ) * elapsedTime * RAD_TO_DEG;
}

int calculateDelay()
{
  double delay = (duration - estimatedTime) / double(path.size()) - 200; // delay per move, give 200ms extra time

  // max stop for 3 seconds have 0.5 sec margin
  if (delay > 2900)
  {
    return 2900;
  }
  else if (delay < 0) {
    return 0; // no time :(
  }
  else
  {
    return int(delay);
  }
}

// set speed of left and right motor
void setMotorSpeeds(double leftSpeed, double rightSpeed)
{
  // set directions
  if (leftSpeed < 0)
  {
    digitalWrite(inLeft1, HIGH);
    digitalWrite(inLeft2, LOW);
  }
  else
  {
    digitalWrite(inLeft1, LOW);
    digitalWrite(inLeft2, HIGH);
  }

  if (rightSpeed < 0)
  {
    digitalWrite(inRight1, LOW);
    digitalWrite(inRight2, HIGH);
  }
  else
  {
    digitalWrite(inRight1, HIGH);
    digitalWrite(inRight2, LOW);
  }

  ledcWrite(leftChannel, abs(leftSpeed));
  ledcWrite(rightChannel, abs(rightSpeed));
  // Serial.println("Left speed: " + String(leftSpeed)); //debug
  // Serial.println("Right speed: " + String(rightSpeed)); //debug
}

// move forward 25cm(half a square)
void moveForward()
{
  if (stupid) {
    setMotorSpeeds(255, 255);
    delay(forwardDuration);
    setMotorSpeeds(0, 0);
    return;
  }

  headingPID.SetTunings(heading_Kp, heading_Ki, heading_Kd);
  motor_setpoint = forwardDist / (diameter * PI) * ticksPerRev; // calculate encoder count needed for the distance
  // Serial.println("Motor Setpoint: " + String(motor_setpoint)); //debug
  heading_setpoint = curr_heading; // keep curr_heading at the start
  // Serial.println("Heading Setpoint: " + String(heading_setpoint)); //debug

  left.clearCount();
  right.clearCount();

  long leftPos = 0;
  long rightPos = 0;

  while (abs((leftPos + rightPos) / 2 - motor_setpoint) > forward_threshold)
  { // average of encoder values
    leftPos = left.getCount() / 2;
    rightPos = right.getCount() / 2;
    //Serial.println("Left Encoder: " + String(leftPos)); //debug
    //Serial.println("Right Encoder: " + String(rightPos)); //debug

    // find speed with PID
    motor_input = (leftPos + rightPos) / 2.0;
    motorPID.Compute();

    // correct curr_heading with PID
    calculateHeading();
    heading_input = yaw;
    //Serial.println("Heading input: " + String(heading_input)); //debug
    // Serial.println("Heading error: " + String(heading_input - heading_setpoint)); //debug
    headingPID.Compute();

    setMotorSpeeds(motor_output + heading_output, motor_output - heading_output);

    //Serial.println("Motor PID output: " + String(motor_output)); //debug
    //Serial.println("Heading PID output: " + String(heading_output)); //debug
  }

  setMotorSpeeds(0, 0); // stop motor
}

// turns according to degrees clockwise(negative is ccw)
void turn(double angle)
{
  // TODO - fix this bad logic
  if (stupid) {
    if (angle  == -90) {
      setMotorSpeeds(-255, 255);
      delay(turn90Duration);
    }
    else if (angle == -180) {
      setMotorSpeeds(-255, 255);
      delay(turn180Duration);
    }
    else if (angle == 90) {
      setMotorSpeeds(255, -255);
      delay(turn90Duration);
    }
    else if (angle == 180) {
      setMotorSpeeds(255, -255);
      delay(turn180Duration);
    }
    setMotorSpeeds(0, 0);
    return;
  }

  turn_setpoint = curr_heading + angle;
  // Serial.println("Turn setpoint: " + String(turn_setpoint)); //debug

  double error = yaw - turn_setpoint;

  while (abs(error) > turn_threshold)
  {

    calculateHeading();

    turn_input = yaw;
    error = turn_input - turn_setpoint;

    //Serial.println("Turn input: " + String(turn_input)); //debug
    //Serial.println("Turn error: " + String(error)); //debug
    turnPID.Compute();

    setMotorSpeeds(turn_output, -turn_output);

    //Serial.println("Turn output: " + String(turn_output)); //debug
  }

  setMotorSpeeds(0, 0); // stop motor
}

int runPath()
{
  
  startTime = millis();

  for (int i : path)
  {
    // Serial.println(i); //debugging

    // hard stop to get time points
    if (millis() - startTime >= duration)
    {
      return 1;
    }

    if (i == 0)
    {
      moveForward();
    }
    else
    {
      turn(i);
    }
    curr_heading += i;

    if (!stupid && useDelay)
      delay(calculateDelay());
  }

  return 2;
}

void setup()
{
  Serial.begin(115200);

  motorPID.SetMode(AUTOMATIC);
  headingPID.SetMode(AUTOMATIC);
  turnPID.SetMode(AUTOMATIC);

  // motor doesn't have enough torque to overcome static friction at about 40 pwm

  motorPID.SetOutputLimits(0, 255);
  headingPID.SetOutputLimits(-255, 255); // derivative terms may mess up with the negative
  turnPID.SetOutputLimits(-255, 255);

  left.attachHalfQuad(leftPinA, leftPinB);
  right.attachHalfQuad(rightPinA, rightPinB);

  // set encoder count to 0
  left.clearCount();
  right.clearCount();

  // set motor wires to motor_output
  pinMode(inLeft1, OUTPUT);
  pinMode(inLeft2, OUTPUT);
  pinMode(inRight1, OUTPUT);
  pinMode(inRight2, OUTPUT);

  // pwm for esp32, 8 bit(0-255)
  ledcSetup(leftChannel, 1000, 8);
  ledcAttachPin(enLeft, leftChannel); // assign left motor enable

  ledcSetup(rightChannel, 1000, 8);
  ledcAttachPin(enRight, rightChannel); // assign right motor enable

  pinMode(statusLED, OUTPUT);
  pinMode(enableButton, INPUT_PULLUP);

  // initialize mpu
  if (!mpu.begin())
  {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  calculateIMUError();
  delay(20);
}

int endState = 0; // state 0 is not completed, 1 is forced end(ran out of time), 2 is successful
int enableState = HIGH; // not enabled yet

void loop()
{

  if (enableState == LOW) {
    if (endState == 0)
    {
      Serial.println("Path beginning, estimated time: " + String(estimatedTime / 1000.0));

      endState = runPath();

      Serial.println("Path Completed, total time: " + String(millis() - startTime));
    }
    else if (endState == 1)
    {
      digitalWrite(statusLED, HIGH);
      delay(200);
      digitalWrite(statusLED, LOW);
      delay(200);
    }
    else if (endState == 2)
    {
      digitalWrite(statusLED, HIGH);
    }
  } else {
    enableState = digitalRead(enableButton);
  }

}

