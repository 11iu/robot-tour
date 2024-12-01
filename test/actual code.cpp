#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

/* Info */
// Motor A is the right motor (when seen from the back), Motor B is the left motor
// 

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

/* Constants */
const int COUNTS_PER_REV = 1320; // for full quad, reduction ratio 30 with 11 ticks per rev
const float WHEEL_DIAMETER = 60.0; // in mm, banebots 2 3/8in wheels
const float TRACK_WIDTH = 149.5; // in mm, distance between the two wheels

/* Objects */
Adafruit_MPU6050 mpu;
double forwardInput, forwardOutput, forwardSetpoint;
double turnInput, turnOutput, turnSetpoint;
PID forwardPid(&forwardInput, &forwardOutput, &forwardSetpoint, 2.0, 0, 0, DIRECT);
PID turnPid(&turnInput, &turnOutput, &turnSetpoint, 1, 0, 0, DIRECT);
ESP32Encoder maEnc;
ESP32Encoder mbEnc;

void move_left(int speed) {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(MB_IN1, HIGH);
    digitalWrite(MB_IN2, LOW);
  } else {
    digitalWrite(MB_IN1, LOW);
    digitalWrite(MB_IN2, HIGH);
    speed *= -1;
  }
  analogWrite(MB_PWM, speed);
}

void move_right(int speed) {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(MA_IN1, LOW);
    digitalWrite(MA_IN2, HIGH);
  } else {
    digitalWrite(MA_IN1, HIGH);
    digitalWrite(MA_IN2, LOW);
    speed *= -1;
  }
  analogWrite(MA_PWM, speed);
}
// go forward 250mm
void forward_half() {
  while (true) {
    forwardSetpoint = 250;
    forwardInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    forwardPid.Compute();
    move_left(forwardOutput);
    move_right(forwardOutput);
}

  }
void setup() {
  Serial.begin(115200);

  // Initialize pins
  pinMode(MA_PWM, OUTPUT);
  pinMode(MA_IN1, OUTPUT);
  pinMode(MA_IN2, OUTPUT);
  pinMode(MB_PWM, OUTPUT);
  pinMode(MB_IN1, OUTPUT);
  pinMode(MB_IN2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // Initialize pid
  forwardPid.SetMode(AUTOMATIC);
  forwardPid.SetOutputLimits(-255, 255);
  forwardPid.SetSampleTime(10);

  // Initialize MPU6050
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("MPU6050 Found!");

  // Initialize encoders
  maEnc.attachFullQuad(MA_ENC_B, MA_ENC_A);
  mbEnc.attachFullQuad(MB_ENC_A, MB_ENC_B);

  // forward_half();
}

void loop() {
  forwardSetpoint = 250;
  forwardInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
  forwardPid.Compute();

  // proportional control to go straight
  float kP = 10.0;
  float straightInput = (maEnc.getCount() - mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
  float straightOutput = kP * straightInput;

  // Serial.println("Straight error = " + String(straightInput));

  move_left(forwardOutput + straightOutput);
  move_right(forwardOutput - straightOutput);
  
  //Serial.println("Encoder count = " + String((int32_t)mbEnc.getCount()) + " " + String((int32_t)maEnc.getCount()));
  Serial.print("PID input:" + String(forwardInput));
  Serial.print(" Pid error:" + String(forwardSetpoint - forwardInput));
  Serial.println("PID output:" + String(forwardOutput));
}