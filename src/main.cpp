#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

/* Info */
// Motor A is the right motor (when seen from the back), Motor B is the left motor
// Error codes: fast blink = MPU6050 not found,

/* Pins */
const int MA_ENC_A = 36;
const int MA_ENC_B = 39;
const int MA_PWM = 32;
const int MA_IN2 = 33;
const int MA_IN1 = 25;
const int MB_ENC_A = 34;
const int MB_ENC_B = 35;
const int MB_PWM = 13;
const int MB_IN2 = 26;
const int MB_IN1 = 27;
const int BUTTON = 4;
const int LED_BUILTIN = 2;

/* Constants */
const int COUNTS_PER_REV = 1320;   // for full quad, reduction ratio 30 with 11 ticks per rev
const float WHEEL_DIAMETER = 60.0; // in mm, banebots 2 3/8in wheels
const float TRACK_WIDTH = 138.5;   // in mm, distance between the two wheels, tested value - actual 149
const int MIN_SPEED = 20;          // pwm value to overcome static friction out of 255

const float SAME_FORWARD_KP = 100.0; // tuned, 128 oscillation
const float SAME_TURN_KP = 8.0;      // tuned, 32 has weird turning
const float FORWARD_CUTOFF = 2;      // int mm, threshold to stop moving forward
const float TURN_CUTOFF = 0.5;       // in mm, threshold to stop turning
const float TURN_GYRO_CUTOFF = 0.1;  // in degrees, threshold to stop turning
// TODO: add time cutoff for forward and turn

/* Variables */
double forwardInput, forwardOutput, forwardSetpoint;
double turnEncoderInput, turnEncoderOutput, turnEncoderSetpoint;
double turnGyroInput, turnGyroOutput, turnGyroSetpoint;
float angle = 0.0;

/* Objects */
Adafruit_MPU6050 mpu;
PID forwardPid(&forwardInput, &forwardOutput, &forwardSetpoint, 2.0, 0.0, 0.0, DIRECT);
PID turnEncoderPid(&turnEncoderInput, &turnEncoderOutput, &turnEncoderSetpoint, 2.0, 0.0, 0.0, DIRECT);
PID turnGyroPid(&turnGyroInput, &turnGyroOutput, &turnGyroSetpoint, 3.0, 0.0, 0.0, DIRECT);
ESP32Encoder maEnc;
ESP32Encoder mbEnc;

void reset_encoders()
{
  maEnc.clearCount();
  mbEnc.clearCount();
}

void move_left(float speed)
{

  if (fabs(speed) < 0.01)
  { // Use a small threshold to compare floats
    digitalWrite(MB_IN1, LOW);
    digitalWrite(MB_IN2, LOW);
  }
  else if (speed > 0)
  {
    digitalWrite(MB_IN1, HIGH);
    digitalWrite(MB_IN2, LOW);
  }
  else
  {
    digitalWrite(MB_IN1, LOW);
    digitalWrite(MB_IN2, HIGH);
    speed *= -1;
  }

  int speed_i = (int)roundf(speed); // deal with -0.5 case where int cast rounding does not work

  // clamp the speed
  if (speed_i < MIN_SPEED)
    speed_i = MIN_SPEED;
  if (speed_i > 255)
    speed_i = 255;

  analogWrite(MB_PWM, speed_i);
}

void move_right(float speed)
{

  if (fabs(speed) < 0.01)
  {
    digitalWrite(MA_IN1, LOW);
    digitalWrite(MA_IN2, LOW);
  }
  else if (speed > 0)
  {
    digitalWrite(MA_IN1, LOW);
    digitalWrite(MA_IN2, HIGH);
  }
  else
  {
    digitalWrite(MA_IN1, HIGH);
    digitalWrite(MA_IN2, LOW);
    speed *= -1;
  }

  int speed_i = (int)roundf(speed);

  // clamp the speed
  if (speed_i < MIN_SPEED)
    speed_i = MIN_SPEED;
  if (speed_i > 255)
    speed_i = 255;

  analogWrite(MA_PWM, speed_i);
}

// go forward 250mm
void forward_half()
{
  reset_encoders();
  forwardSetpoint = 250.0;
  do
  {
    forwardInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    forwardPid.Compute();

    // proportional control to go straight
    float sameForwardInput = (maEnc.getCount() - mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameForwardOutput = SAME_FORWARD_KP * sameForwardInput;

    move_left(forwardOutput + sameForwardOutput);
    move_right(forwardOutput - sameForwardOutput);

    // Serial.print("lEnc:" + String((int32_t)mbEnc.getCount()));
    // Serial.print(" rEnc:" + String((int32_t)maEnc.getCount()));
    // Serial.print(" Straight er:" + String(sameForwardInput, 2));
    // Serial.print(" PID in:" + String(forwardInput, 2));
    // Serial.print(" Pid er:" + String(forwardSetpoint - forwardInput, 2));
    // Serial.println(" PID ou:" + String(forwardOutput, 2));
    delay(2); // ensure loop takes longer than pid update rate
  } while (abs(forwardSetpoint - forwardInput) > FORWARD_CUTOFF);
  move_left(0);
  move_right(0);
}

// based on encoder only, p to maintain same motor rotation amounts
void turn_right_encoder()
{
  reset_encoders();
  turnEncoderSetpoint = TRACK_WIDTH * PI / 4.0 - 4.4; // tuned

  do
  {
    turnEncoderInput = (-maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    turnEncoderPid.Compute();

    // proportional control for both motors to turn the same amount
    float sameTurnInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameTurnOutput = SAME_TURN_KP * sameTurnInput;

    move_left(turnEncoderOutput - sameTurnOutput);
    move_right(-turnEncoderOutput - sameTurnOutput);

    // Serial.print("Same er:" + String(sameTurnInput, 2));
    // Serial.print(" PID in:" + String(turnEncoderInput, 2));
    // Serial.print(" PID er:" + String(turnEncoderSetpoint - turnEncoderInput, 2));
    // Serial.println(" PID ou:" + String(turnEncoderOutput, 2));
    delay(2); // ensure loop takes longer than pid update rate
  } while (abs(turnEncoderSetpoint - turnEncoderInput) > TURN_CUTOFF);
  move_left(0);
  move_right(0);
}

// based on encoder only, p to maintain same motor rotation amounts
void turn_left_encoder()
{
  reset_encoders();
  turnEncoderSetpoint = TRACK_WIDTH * PI / 4.0 - 4.4; // tuned

  do
  {
    turnEncoderInput = (maEnc.getCount() - mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    turnEncoderPid.Compute();

    // proportional control for both motors to turn the same amount
    float sameTurnInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameTurnOutput = SAME_TURN_KP * sameTurnInput;

    move_left(-turnEncoderOutput - sameTurnOutput);
    move_right(turnEncoderOutput - sameTurnOutput);

    // Serial.print("Same er:" + String(sameTurnInput, 2));
    // Serial.print(" PID in:" + String(turnEncoderInput, 2));
    // Serial.print(" PID er:" + String(turnEncoderSetpoint - turnEncoderInput, 2));
    // Serial.println(" PID ou:" + String(turnEncoderOutput, 2));
    delay(2); // ensure loop takes longer than pid update rate
  } while (abs(turnEncoderSetpoint - turnEncoderInput) > TURN_CUTOFF);
  move_left(0);
  move_right(0);
}

// turn right based on gyro
void turn_right_gyro()
{
  turnGyroSetpoint = 84.5; // FIXME: 90 always overshoots
  float start_angle = angle;

  uint16_t lastReading = millis();

  do
  {
    uint16_t now = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    angle -= g.gyro.z * 180 / PI * (now - lastReading) / 1000.0; // subtract to reverse gyro direction
    lastReading = now;

    turnGyroInput = angle - start_angle;
    turnGyroPid.Compute();

    move_left(turnGyroOutput);
    move_right(-turnGyroOutput);

    // Serial.print("Same er:" + String(sameTurnInput, 2));
    // Serial.print(" PID in:" + String(turnEncoderInput, 2));
    // Serial.print(" PID er:" + String(turnEncoderSetpoint - turnEncoderInput, 2));
    // Serial.println(" PID ou:" + String(turnEncoderOutput, 2));
    delay(1); // ensure loop takes longer than pid update rate
  } while (abs(turnGyroSetpoint - turnGyroInput) > TURN_GYRO_CUTOFF);
  move_left(0);
  move_right(0);
}

// turn left based on gyro
void turn_left_gyro()
{
  turnGyroSetpoint = -84.5; // FIXME: 90 always overshoots
  float start_angle = angle;
  uint16_t lastReading = millis();

  do
  {
    uint16_t now = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    angle -= g.gyro.z * 180 / PI * (now - lastReading) / 1000.0; // subtract to reverse gyro direction
    lastReading = now;

    turnGyroInput = angle - start_angle;
    turnGyroPid.Compute();

    move_left(turnGyroOutput);
    move_right(-turnGyroOutput);

    // Serial.print("Same er:" + String(sameTurnInput, 2));
    // Serial.print(" PID in:" + String(turnEncoderInput, 2));
    // Serial.print(" PID er:" + String(turnEncoderSetpoint - turnEncoderInput, 2));
    // Serial.println(" PID ou:" + String(turnEncoderOutput, 2));
    delay(1); // ensure loop takes longer than pid update rate
  } while (abs(turnGyroSetpoint - turnGyroInput) > TURN_GYRO_CUTOFF);
  move_left(0);
  move_right(0);
}

void setup()
{
  Serial.begin(115200);

  // Initialize pins
  pinMode(MA_PWM, OUTPUT);
  pinMode(MA_IN1, OUTPUT);
  pinMode(MA_IN2, OUTPUT);
  pinMode(MB_PWM, OUTPUT);
  pinMode(MB_IN1, OUTPUT);
  pinMode(MB_IN2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize pid
  forwardPid.SetMode(AUTOMATIC);
  forwardPid.SetOutputLimits(-255, 255);
  forwardPid.SetSampleTime(1); // don't increase or breaks
  turnEncoderPid.SetMode(AUTOMATIC);
  turnEncoderPid.SetOutputLimits(-255, 255);
  turnEncoderPid.SetSampleTime(1); // don't increase or breaks
  turnGyroPid.SetMode(AUTOMATIC);
  turnGyroPid.SetOutputLimits(-255, 255);
  turnGyroPid.SetSampleTime(1); // don't increase or breaks

  // Initialize MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Initialize encoders
  maEnc.attachFullQuad(MA_ENC_B, MA_ENC_A);
  mbEnc.attachFullQuad(MB_ENC_A, MB_ENC_B);
}

bool start_moving = false;

void loop()
{

  // press button to move
  if (digitalRead(BUTTON) == LOW && !start_moving)
  {
    start_moving = true;
    maEnc.clearCount();
    mbEnc.clearCount();
    move_left(0);
    move_right(0);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000); // time for finger to leave
  }

  if (start_moving)
  {
    /*###### COMPETITION CODE GOES HERE ########*/
    forward_half();
    delay(200);
    forward_half();
    delay(200);
    turn_right_encoder();
    delay(200);
    turn_right_encoder();
    delay(200);
    forward_half();
    delay(200);
    forward_half();
    delay(200);
    turn_left_encoder();
    delay(200);
    turn_left_encoder();
    delay(200);
    start_moving = false;
  }
}