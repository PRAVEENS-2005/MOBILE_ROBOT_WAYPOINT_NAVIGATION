// =======================================================
// Arduino Mega 2560 - ROBOT CONTROLLER WITH CALIBRATION
// FIXED VERSION - Proper float formatting
// =======================================================
#include <Wire.h>
#include <MPU6050.h>

// ------------------ PINS ------------------
#define MOTOR_LEFT_PWM   5
#define MOTOR_LEFT_DIR   4
#define MOTOR_RIGHT_PWM  6
#define MOTOR_RIGHT_DIR  7
#define ENCODER_LEFT_A   18
#define ENCODER_LEFT_B   19
#define ENCODER_RIGHT_A   2
#define ENCODER_RIGHT_B   3

// ------------------ ROBOT PARAMS ------------------
#define WHEEL_DIAMETER   0.127    // meters (actual measurement)
#define WHEEL_BASE       0.46     // meters (actual measurement)
#define LEFT_PPR         2320     // Pulses per revolution
#define RIGHT_PPR        2350     // Pulses per revolution
#define MAX_LINEAR       0.5      // m/s
#define MAX_ANGULAR      2.0      // rad/s
#define MIN_PWM          60       // Dead-band compensation

// ------------------ CALIBRATION FACTORS ------------------
#define LINEAR_CORRECTION    1.00    // Multiply linear distance
#define ROTATION_CORRECTION  1.00    // Multiply rotation angle
#define LEFT_WHEEL_FACTOR    1.00    // Individual wheel correction
#define RIGHT_WHEEL_FACTOR   1.00    // Individual wheel correction

// ------------------ GLOBALS ------------------
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long prevLeftCount = 0;
long prevRightCount = 0;

float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;

float cmd_vel_linear = 0.0;
float cmd_vel_angular = 0.0;

unsigned long lastCmdTime = 0;
unsigned long lastControlTime = 0;

MPU6050 imu;
bool imu_ok = false;
float gyro_z_rad = 0.0;

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
 
  Serial.println("========================================");
  Serial.println("ROBOT CONTROLLER v3.1 - FIXED");
  Serial.println("========================================");
 
  // Motors
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  setMotorSpeed(0, 0);
  Serial.println("Motors: OK");
 
  // Encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);
  Serial.println("Encoders: OK");
 
  // IMU
  Wire.begin();
  imu.initialize();
  delay(100);
  if (imu.testConnection()) {
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    imu_ok = true;
    Serial.println("IMU: OK");
  } else {
    Serial.println("IMU: NOT FOUND");
  }
 
  // Print calibration factors
  Serial.println("----------------------------------------");
  Serial.println("CALIBRATION FACTORS:");
  Serial.print("  Linear Correction: "); Serial.println(LINEAR_CORRECTION, 3);
  Serial.print("  Rotation Correction: "); Serial.println(ROTATION_CORRECTION, 3);
  Serial.print("  Left Wheel Factor: "); Serial.println(LEFT_WHEEL_FACTOR, 3);
  Serial.print("  Right Wheel Factor: "); Serial.println(RIGHT_WHEEL_FACTOR, 3);
  Serial.println("----------------------------------------");
 
  Serial.println("READY - Send CMD_VEL:linear,angular");
  Serial.println("========================================");
 
  lastCmdTime = millis();
  lastControlTime = millis();
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  unsigned long now = millis();
 
  // Process serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
   
    if (cmd.startsWith("CMD_VEL:")) {
      int comma = cmd.indexOf(',');
      if (comma > 0) {
        cmd_vel_linear = constrain(cmd.substring(8, comma).toFloat(), -MAX_LINEAR, MAX_LINEAR);
        cmd_vel_angular = constrain(cmd.substring(comma + 1).toFloat(), -MAX_ANGULAR, MAX_ANGULAR);
        lastCmdTime = now;
      }
    }
    else if (cmd == "STOP") {
      cmd_vel_linear = 0.0;
      cmd_vel_angular = 0.0;
      setMotorSpeed(0, 0);
      Serial.println("ACK:STOP");
    }
    else if (cmd == "RESET_ODOM") {
      x_pos = 0.0;
      y_pos = 0.0;
      theta = 0.0;
      noInterrupts();
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      prevLeftCount = 0;
      prevRightCount = 0;
      interrupts();
      Serial.println("ACK:RESET_ODOM");
    }
    else if (cmd == "GET_ENCODERS") {
      Serial.print("ENC:");
      Serial.print(leftEncoderCount);
      Serial.print(",");
      Serial.println(rightEncoderCount);
    }
    else if (cmd == "GET_CALIBRATION") {
      Serial.println("CALIBRATION:");
      Serial.print("  LINEAR_CORRECTION="); Serial.println(LINEAR_CORRECTION, 4);
      Serial.print("  ROTATION_CORRECTION="); Serial.println(ROTATION_CORRECTION, 4);
      Serial.print("  LEFT_WHEEL_FACTOR="); Serial.println(LEFT_WHEEL_FACTOR, 4);
      Serial.print("  RIGHT_WHEEL_FACTOR="); Serial.println(RIGHT_WHEEL_FACTOR, 4);
    }
  }
 
  // Timeout (2 seconds)
  if (now - lastCmdTime > 2000) {
    if (cmd_vel_linear != 0.0 || cmd_vel_angular != 0.0) {
      cmd_vel_linear = 0.0;
      cmd_vel_angular = 0.0;
    }
  }
 
  // Control loop at 20Hz
  if (now - lastControlTime >= 50) {
    lastControlTime = now;
   
    updateIMU();
    updateMotorControl();
    updateOdometry();
    sendOdometry();
  }
}

// =======================================================
// ENCODER ISRs
// =======================================================
void leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

// =======================================================
// MOTOR CONTROL
// =======================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    analogWrite(MOTOR_LEFT_PWM, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    analogWrite(MOTOR_LEFT_PWM, constrain(-leftSpeed, 0, 255));
  }
 
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    analogWrite(MOTOR_RIGHT_PWM, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, constrain(-rightSpeed, 0, 255));
  }
}

void updateMotorControl() {
  float v_left = cmd_vel_linear - (cmd_vel_angular * WHEEL_BASE / 2.0);
  float v_right = cmd_vel_linear + (cmd_vel_angular * WHEEL_BASE / 2.0);
 
  int pwm_left = (int)((v_left / MAX_LINEAR) * 255.0);
  int pwm_right = (int)((v_right / MAX_LINEAR) * 255.0);
 
  // Dead-band compensation
  if (pwm_left > 0 && pwm_left < MIN_PWM) pwm_left = MIN_PWM;
  if (pwm_left < 0 && pwm_left > -MIN_PWM) pwm_left = -MIN_PWM;
  if (pwm_right > 0 && pwm_right < MIN_PWM) pwm_right = MIN_PWM;
  if (pwm_right < 0 && pwm_right > -MIN_PWM) pwm_right = -MIN_PWM;
 
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
 
  setMotorSpeed(pwm_left, pwm_right);
}

// =======================================================
// IMU
// =======================================================
void updateIMU() {
  if (!imu_ok) return;
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyro_z_rad = (gz / 131.0) * (PI / 180.0);
}

// =======================================================
// ODOMETRY - WITH CALIBRATION CORRECTIONS
// =======================================================
void updateOdometry() {
  static unsigned long lastOdomTime = 0;
  unsigned long now = millis();
  float dt = (now - lastOdomTime) / 1000.0;
  lastOdomTime = now;
 
  if (dt <= 0.0 || dt > 1.0) return; // Skip first run or if time is weird
 
  noInterrupts();
  long leftDelta = leftEncoderCount - prevLeftCount;
  long rightDelta = rightEncoderCount - prevRightCount;
  prevLeftCount = leftEncoderCount;
  prevRightCount = rightEncoderCount;
  interrupts();
 
  // Calculate wheel circumference
  float circumference = PI * WHEEL_DIAMETER;
 
  // Convert encoder ticks to distance WITH individual wheel correction
  float leftDist = (leftDelta / (float)LEFT_PPR) * circumference * LEFT_WHEEL_FACTOR;
  float rightDist = (rightDelta / (float)RIGHT_PPR) * circumference * RIGHT_WHEEL_FACTOR;
 
  // Calculate center distance and rotation
  float centerDist = (leftDist + rightDist) / 2.0;
  float dTheta = (rightDist - leftDist) / WHEEL_BASE;
 
  // *** APPLY CALIBRATION CORRECTIONS ***
  centerDist *= LINEAR_CORRECTION;
  dTheta *= ROTATION_CORRECTION;
 
  // Optional gyro fusion
  if (imu_ok) {
    dTheta = 0.5 * dTheta + 0.5 * gyro_z_rad * dt;
  }
 
  // Update pose
  theta += dTheta;
  x_pos += centerDist * cos(theta);
  y_pos += centerDist * sin(theta);
 
  // Normalize theta
  while (theta > PI) theta -= 2.0 * PI;
  while (theta < -PI) theta += 2.0 * PI;
}

// =======================================================
// OUTPUT - FIXED: Proper float formatting with dtostrf
// =======================================================
void sendOdometry() {
  // Calculate velocities
  static unsigned long lastSendTime = 0;
  unsigned long now = millis();
  float dt = (now - lastSendTime) / 1000.0;
  lastSendTime = now;
 
  if (dt <= 0.0) return;
 
  static float last_x = 0.0;
  static float last_y = 0.0;
  static float last_theta = 0.0;
 
  float v_linear = sqrt(pow(x_pos - last_x, 2) + pow(y_pos - last_y, 2)) / dt;
  float v_angular = (theta - last_theta) / dt;
 
  last_x = x_pos;
  last_y = y_pos;
  last_theta = theta;
 
  // Buffer for float to string conversion
  char buf[12];
 
  // Send odometry with PROPER float formatting
  // ODOM:x,y,theta,v_lin,v_ang,gyro_z,enc_l,enc_r
  Serial.print("ODOM:");
 
  dtostrf(x_pos, 8, 4, buf);
  Serial.print(buf); Serial.print(",");
 
  dtostrf(y_pos, 8, 4, buf);
  Serial.print(buf); Serial.print(",");
 
  dtostrf(theta, 8, 4, buf);
  Serial.print(buf); Serial.print(",");
 
  dtostrf(v_linear, 8, 4, buf);
  Serial.print(buf); Serial.print(",");
 
  dtostrf(v_angular, 8, 4, buf);
  Serial.print(buf); Serial.print(",");
 
  dtostrf(gyro_z_rad, 8, 4, buf);
  Serial.print(buf); Serial.print(",");
 
  Serial.print(leftEncoderCount); Serial.print(",");
  Serial.println(rightEncoderCount);
}