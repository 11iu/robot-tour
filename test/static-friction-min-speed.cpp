#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
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

  // Initialize encoders
  maEnc.attachFullQuad(MA_ENC_B, MA_ENC_A);
  mbEnc.attachFullQuad(MB_ENC_A, MB_ENC_B);
}

void loop() {
  // set speed based on keyboard input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int speed = input.toInt();

    // Print the input value to the serial monitor
    Serial.print("Input speed: ");
    Serial.println(speed);

    // Set the motor speed
    move_left(speed);
    move_right(speed);
  }

  // Add a small delay
  delay(10);
}