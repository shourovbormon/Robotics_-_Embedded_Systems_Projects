/*
 * ESP32 Line Following Robot
 * Features: Gas Start, Hand-Swipe Start (New Sonar), Auto-Dustbin, Instant Stop
 */

#include <ESP32Servo.h>
#include <NewPing.h>

// ==================== PIN MAP ====================
// --- LFR Pins ---
const int irPins[6] = { 33, 32, 35, 34, 39, 36 };
#define LEFT_IN1 17
#define LEFT_IN2 16
#define RIGHT_IN1 19
#define RIGHT_IN2 18

// --- Dustbin Pins ---
#define DUST_TRIG 27   
#define DUST_ECHO 26   
#define SERVO_PIN 14  
#define SERVO_PIN1 2

// --- Start Sensors (Gas & New Sonar) ---
#define GAS_PIN 13    
#define START_TRIG 25  // NEW: Start Sensor Trigger
#define START_ECHO 12  // NEW: Changed from 33 to 21 to prevent IR conflict!

// ==================== TUNING ====================
const int THRESHOLD = 2000;

const int maxSpeed = 150;
const int baseSpeed = 100;

float Kp = 3.0;
float Ki = 0.0;
float Kd = 10.0;

// ==================== GLOBALS ====================
// --- LFR Globals ---
float error = 0;
float lastError = 0;
float integral = 0;

// --- State Machine Globals ---
bool isActivated = false;             
bool robotFinished = false;           

// --- Sonar Setup ---
#define MAX_DISTANCE 200
NewPing sonarDustbin(DUST_TRIG, DUST_ECHO, MAX_DISTANCE);
NewPing sonarStart(START_TRIG, START_ECHO, MAX_DISTANCE);

Servo dustH;
Servo dustB;
unsigned long dustbinOpenTime = 0;
unsigned long lastDustbinPing = 0; 
unsigned long lastStartPing = 25; // Offset by 25ms to prevent sensor cross-talk
bool isDustbinOpen = false;
const unsigned long openDuration = 3000;  
bool a  = true;
// ==================== MOTOR CONTROL ====================
void setLeftMotor(int speed) {
  speed = constrain(speed, -maxSpeed, maxSpeed);
  if (speed >= 0) {
    analogWrite(LEFT_IN1, speed);
    analogWrite(LEFT_IN2, 0);
  } else {
    analogWrite(LEFT_IN1, 0);
    analogWrite(LEFT_IN2, -speed);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -maxSpeed, maxSpeed);
  if (speed >= 0) {
    analogWrite(RIGHT_IN1, speed);
    analogWrite(RIGHT_IN2, 0);
  } else {
    analogWrite(RIGHT_IN1, 0);
    analogWrite(RIGHT_IN2, -speed);
  }
}

void motor(int left, int right) {
  setLeftMotor(left);
  setRightMotor(right);
}

void stopMotors() {
  motor(0, 0);
}

// ==================== SENSOR READ ====================
void calculateError() {
  int activeSensors = 0;
  long weightedSum = 0;

  for (int i = 0; i < 6; i++) {
    int v = analogRead(irPins[i]);
    int isLine = (v > THRESHOLD) ? 1 : 0;

    activeSensors += isLine;
    weightedSum += isLine * (i * 1000);
  }

  // --- INSTANT Stop Condition ---
  if (activeSensors == 6) {
    robotFinished = true; 
    isActivated = false;  
  }

  // --- Normal PID Math ---
  if (activeSensors != 0) {
    float position = (float)weightedSum / (float)activeSensors;
    error = (position - 2500.0) / 100.0;
  } else {
    if (lastError > 0) error = 25.0;
    else if (lastError < 0) error = -25.0;
    else error = 0;
  }
}

// ==================== PID CONTROL ====================
void pidLFR() {
  calculateError();

  if (robotFinished) {
    stopMotors();
    return;
  }

  float P = error;
  integral += error;
  float D = error - lastError;
  integral = constrain(integral, -100, 100);

  float pidValue = (Kp * P) + (Ki * integral) + (Kd * D);
  lastError = error;

  int leftSpeed = baseSpeed + pidValue;
  int rightSpeed = baseSpeed - pidValue;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motor(leftSpeed, rightSpeed);
  delay(20);  
}

// ==================== DUSTBIN CONTROL ====================
void handleDustbin() {
  if (millis() - lastDustbinPing >= 50) {
    lastDustbinPing = millis(); 
    int sensorS = sonarDustbin.ping_cm();

    if (sensorS > 0 && sensorS < 15) {
      if (!isDustbinOpen) {
        dustH.write(30);  
        isDustbinOpen = true;
      }
      dustbinOpenTime = millis();
    }
  }

  if (isDustbinOpen && (millis() - dustbinOpenTime >= openDuration)) {
    dustH.write(95);  
    isDustbinOpen = false;
  }
}

// ==================== NEW LFR START SENSOR ====================
void handleStartSensor() {
  // Check the new ultrasonic sensor every 50ms
  if (millis() - lastStartPing >= 50) {
    lastStartPing = millis();
    int startDist = sonarStart.ping_cm();

    // If an object is between 1cm and 3cm, turn ON the robot
    if (startDist > 0 && startDist <= 3 && !robotFinished) {
      isActivated = true;
    }
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- ESP32 Robot Starting ---");

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  dustH.attach(SERVO_PIN);
  dustB.attach(SERVO_PIN1);
  dustH.write(95);  
  pinMode(GAS_PIN, INPUT);

  delay(1500);
}

// ==================== MAIN LOOP ====================
void loop() {
  // 1. Dustbin always runs smoothly
  handleDustbin();

  // 2. Check the new Hand-Swipe Start Sensor
  handleStartSensor();

  // 3. Watch for Gas Trigger
  if (digitalRead(GAS_PIN) == LOW && !robotFinished) {
    isActivated = true; 
  }

  // 4. Decide what the motors should do
  if (robotFinished) {
    stopMotors(); // End of the line reached, stay stopped forever
    if (a == true) {
      dustB.write(35);  // Full speed forward
      delay(1000);      // Let it spin for 1 second

      // 2. STOP
      dustB.write(90);  // Stop command
      dustH.write(30);
      delay(2000);

      dustB.write(145);  // Full speed forward
      delay(1000);       // Let it spin for 1 second

      // 2. STOP
      dustB.write(90);  // Stop command
      dustH.write(95);
      delay(2000);
      a = false;
    }
  } 
  else if (isActivated) {
    pidLFR();     // Gas or Hand triggered the robot, keep driving!
  } 
  else {
    stopMotors(); // Waiting for the very first gas or hand trigger
  }
}