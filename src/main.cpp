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
const int COUNTS_PER_REV = 1320; // for full quad, reduction ratio 30 with 11 ticks per rev
const float WHEEL_DIAMETER = 60.0; // in mm, banebots 2 3/8in wheels
const float TRACK_WIDTH = 142.5; // in mm, distance between the two wheels, tested value - actual 149
const int MIN_SPEED = 17; // pwm value to overcome static friction out of 255
const float SAME_FORWARD_KP = 64.0; // tuned, 128 oscillation
const float SAME_TURN_KP = 8.0; // tuned, 32 has weird turning
const float FORWARD_CUTOFF = 2; // int mm, threshold to stop moving forward
const float TURN_CUTOFF = 0.5; // in mm, threshold to stop turning
const float TURN_GYRO_CUTOFF = 1.0; // in degrees, threshold to stop turning
//TODO: add time cutoff for forward and turn

/* Variables */
double forwardInput, forwardOutput, forwardSetpoint;
double turnInput, turnOutput, turnSetpoint;
double turnGyroInput, turnGyroOutput, turnGyroSetpoint;
float angle = 0.0;

/* Objects */
Adafruit_MPU6050 mpu;
PID forwardPid(&forwardInput, &forwardOutput, &forwardSetpoint, 2.0, 0.0, 0.0, DIRECT);
PID turnPid(&turnInput, &turnOutput, &turnSetpoint, 2.0, 0.0, 0.0, DIRECT);
PID turnGyroPid(&turnGyroInput, &turnGyroOutput, &turnGyroSetpoint, 4.0, 0.0, 0.0, DIRECT);
ESP32Encoder maEnc;
ESP32Encoder mbEnc;

TaskHandle_t GyroTask;
TaskHandle_t CommTask;
SemaphoreHandle_t angleMutex;

// Function to handle gyro integration in a separate task
void gyroTask(void *pvParameters) {
  sensors_event_t a, g, temp;
  while (true) {
    mpu.getEvent(&a, &g, &temp);
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    angle += g.gyro.z;
    xSemaphoreGive(angleMutex);
    delay(1); // Small delay to prevent task from hogging the CPU
  }
}

// Function to handle communication in a separate task
void commTask(void *pvParameters) {
  while (true) {
    // Handle communication (e.g., Wi-Fi, Bluetooth, Serial)
    // Example: Serial communication
    if (Serial.available()) {
      String message = Serial.readString();
      // Process the message
    }
    delay(10); // Adjust delay as needed
  }
}

void reset_encoders() {
  maEnc.clearCount();
  mbEnc.clearCount();
}

void move_left(float speed) {

  if (fabs(speed) < 0.01) { // Use a small threshold to compare floats
    digitalWrite(MB_IN1, LOW);
    digitalWrite(MB_IN2, LOW);
  } else if (speed > 0) {
    digitalWrite(MB_IN1, HIGH);
    digitalWrite(MB_IN2, LOW);
  } else {
    digitalWrite(MB_IN1, LOW);
    digitalWrite(MB_IN2, HIGH);
    speed *= -1;
  }

  int speed_i = (int) roundf(speed); // deal with -0.5 case where int cast rounding does not work

  // clamp the speed
  if (speed_i < MIN_SPEED) speed_i = MIN_SPEED;
  if (speed_i > 255) speed_i = 255;

  analogWrite(MB_PWM, speed_i);
}

void move_right(float speed) {

  if (fabs(speed) < 0.01) {
    digitalWrite(MA_IN1, LOW);
    digitalWrite(MA_IN2, LOW);
  } else if (speed > 0) {
    digitalWrite(MA_IN1, LOW);
    digitalWrite(MA_IN2, HIGH);
  } else {
    digitalWrite(MA_IN1, HIGH);
    digitalWrite(MA_IN2, LOW);
    speed *= -1;
  }

  int speed_i = (int) roundf(speed);

  // clamp the speed
  if (speed_i < MIN_SPEED) speed_i = MIN_SPEED;
  if (speed_i > 255) speed_i = 255;

  analogWrite(MA_PWM, speed_i);
}

// go forward 250mm
void forward_half() {
  reset_encoders();
  forwardSetpoint = 250.0;
  do {
    forwardInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    forwardPid.Compute();

    // proportional control to go straight
    float sameForwardInput = (maEnc.getCount() - mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameForwardOutput = SAME_FORWARD_KP * sameForwardInput;

    move_left(forwardOutput + sameForwardOutput);
    move_right(forwardOutput - sameForwardOutput);

    Serial.print("lEnc:" + String((int32_t)mbEnc.getCount()));
    Serial.print(" rEnc:" + String((int32_t)maEnc.getCount()));
    Serial.print(" Straight er:" + String(sameForwardInput, 2));
    Serial.print(" PID in:" + String(forwardInput, 2));
    Serial.print(" Pid er:" + String(forwardSetpoint - forwardInput, 2));
    Serial.println(" PID ou:" + String(forwardOutput, 2));
    delay(1); // ensure loop takes longer than pid update rate
  } while (abs(forwardSetpoint - forwardInput) > FORWARD_CUTOFF);
  move_left(0);
  move_right(0);
}

// based on encoder only, p to maintain same motor rotation amounts
void turn_right() {
  reset_encoders();
  turnSetpoint = TRACK_WIDTH * PI / 4.0;
  do {
    turnInput = (-maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    turnPid.Compute();

    // proportional control for both motors to turn the same amount
    float sameTurnInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameTurnOutput = SAME_TURN_KP * sameTurnInput;

    move_left(turnOutput - sameTurnOutput);
    move_right(-turnOutput - sameTurnOutput);

    Serial.print("Same er:" + String(sameTurnInput, 2));
    Serial.print(" PID in:" + String(turnInput, 2));
    Serial.print(" PID er:" + String(turnSetpoint - turnInput, 2));
    Serial.println(" PID ou:" + String(turnOutput, 2));
    delay(1); // ensure loop takes longer than pid update rate
  } while (abs(turnSetpoint - turnInput) > TURN_CUTOFF);
  move_left(0);
  move_right(0);
}

// based on encoder only, p to maintain same motor rotation amounts
void turn_left() {
  reset_encoders();
  turnSetpoint = TRACK_WIDTH * PI / 4.0;
  do {
    turnInput = (maEnc.getCount() - mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    turnPid.Compute();

    // proportional control for both motors to turn the same amount
    float sameTurnInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameTurnOutput = SAME_TURN_KP * sameTurnInput;

    move_left(-turnOutput - sameTurnOutput);
    move_right(turnOutput - sameTurnOutput);

    // Serial.print("Same er:" + String(sameTurnInput, 2));
    // Serial.print(" PID in:" + String(turnInput, 2));
    // Serial.print(" PID er:" + String(turnSetpoint - turnInput, 2));
    // Serial.println(" PID ou:" + String(turnOutput, 2));
    delay(1); // ensure loop takes longer than pid update rate
  } while (abs(turnSetpoint - turnInput) > TURN_CUTOFF);
  move_left(0);
  move_right(0);
}

// turn right based on gyro
void turn_right_gyro() {
  reset_encoders();
  turnSetpoint = 90;
  do {
    turnGyroInput = angle;
    turnGyroPid.Compute();

    // proportional control for both motors to turn the same amount
    float sameTurnInput = (maEnc.getCount() + mbEnc.getCount()) / 2.0 / COUNTS_PER_REV * WHEEL_DIAMETER * PI;
    float sameTurnOutput = SAME_TURN_KP * sameTurnInput;

    move_left(turnGyroOutput - sameTurnOutput);
    move_right(-turnGyroOutput - sameTurnOutput);

    // Serial.print("Same er:" + String(sameTurnInput, 2));
    // Serial.print(" PID in:" + String(turnInput, 2));
    // Serial.print(" PID er:" + String(turnSetpoint - turnInput, 2));
    // Serial.println(" PID ou:" + String(turnOutput, 2));
    delay(1); // ensure loop takes longer than pid update rate
  } while (abs(turnGyroSetpoint - turnGyroInput) > TURN_GYRO_CUTOFF);
  move_left(0);
  move_right(0);
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
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize pid
  forwardPid.SetMode(AUTOMATIC);
  forwardPid.SetOutputLimits(-255, 255);
  forwardPid.SetSampleTime(1); // don't increase or breaks
  turnPid.SetMode(AUTOMATIC);
  turnPid.SetOutputLimits(-255, 255);
  turnPid.SetSampleTime(1); // don't increase or breaks

  //Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
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

  // Create a mutex for the angle variable
  angleMutex = xSemaphoreCreateMutex();

  // Create a task for gyro integration on core 1
  xTaskCreatePinnedToCore(
    gyroTask,   // Function to be called
    "GyroTask", // Name of the task
    10000,      // Stack size (bytes)
    NULL,       // Parameter to pass
    1,          // Task priority
    &GyroTask,  // Task handle
    1           // Core to run the task on (0 or 1)
  );

  // Create a task for communication handling on core 1
  xTaskCreatePinnedToCore(
    commTask,   // Function to be called
    "CommTask", // Name of the task
    10000,      // Stack size (bytes)
    NULL,       // Parameter to pass
    1,          // Task priority
    &CommTask,  // Task handle
    1           // Core to run the task on (0 or 1)
  );
}

bool start_moving = false;

void loop() {

  // press button to move
  if (digitalRead(BUTTON) == LOW && !start_moving) {
    start_moving = true;
    maEnc.clearCount();
    mbEnc.clearCount();
    move_left(0);
    move_right(0);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000); // time for finger to leave
  }

  if (start_moving) {
    // Access the angle variable safely
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    float currentAngle = angle;
    xSemaphoreGive(angleMutex);

    // Use currentAngle as needed
    // forward_half();
    // delay(1000);
    // turn_right();
    // delay(1000);
    // turn_right();
    // delay(1000);
    // forward_half();
    // delay(1000);
    // turn_left();
    // delay(1000);
    // turn_left();
    // delay(1000);
    turn_left();
    start_moving = false;
  }
  
}