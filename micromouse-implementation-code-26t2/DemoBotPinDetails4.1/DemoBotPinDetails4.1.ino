#include <digitalWriteFast.h>
#include <Wire.h>
#include <VL6180X.h>
#include <MPU6050_light.h>
#include "DualEncoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

// ================ CONSTANTS ==================
#define PI 3.1415926535897932384626433832795
#define WHEEL_DIAMETER 32.0f
#define WHEEL_RADIUS 16.0f
#define WHEEL_BASE 90.0f
#define PWM_MAX_SPEED 250
#define PWM_MIN_SPEED 15  // TUNE Try 11-14 baseline first then tune thetaDead then this again
#define PWM_THRESHOLD 30

#define WALL_DISTANCE_THRESHOLD 95
#define WALL_TARGET_DISTANCE 54
#define SIDE_WALL_DISTANCE_THRESHOLD 80
#define SIDE_WALL_TARGET_DISTANCE 56

// Pin definitions
#define MOT1PWM 11 // LEFT
#define MOT1DIR 12 // LEFT
#define MOT2PWM 9  // RIGHT
#define MOT2DIR 10 // RIGHT
#define ENC_A_L 2
#define ENC_B_L 7
#define ENC_A_R 3
#define ENC_B_R 8

// =================== OBJECTS ===================
mtrn3100::Motor motorLeft(MOT1PWM, MOT1DIR);
mtrn3100::Motor motorRight(MOT2PWM, MOT2DIR);
mtrn3100::DualEncoder dualEncoder(ENC_A_L, ENC_B_L, ENC_A_R, ENC_B_R);
VL6180X sensorLeft, sensorFront, sensorRight;
MPU6050 mpu(Wire);

// PID Controllers
mtrn3100::PIDController pidStraight(1.8, 0.01, 0.02);
mtrn3100::PIDController pidThetaStraight(5.00, 0.00, 0.00);
mtrn3100::PIDController pidTheta(2.00, 0.20, 0.01);
mtrn3100::PIDController pidWallFollowLeft(2.00, 0.02, 0);
mtrn3100::PIDController pidWallFollowRight(2.00, 0.02, 0);

// Sensor pins
constexpr int sensorPins[] = {A0, A1, A2}; // left, front, right
constexpr byte lidar_addresses[] = {0x54, 0x56, 0x58};

// =================== VARIABLES ===================
// Motor control
int pwmLeft = 0, pwmRight = 0;
int previousPWMForward = 0, previousPWMTurn = 0;
const int maxDeltaPWM = 50;

// Timing and control
unsigned long startTime = 0;
const float distDead = 5.0f;
const float thetaDead = 1.0f; // TUNE up to 1.0f
float referenceAngle = 0;

// Lidar data
int currentSensor = 0;
int distances[3] = {255, 255, 255}; // left, front, right
int wallTargetDist = WALL_TARGET_DISTANCE;

enum MoveType { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, IDLE, SETTLING };

// Movement configuration
const float distanceF = 175.0f; // TUNE, old:175
const float turnAngle = 90.0f;
const float wallSmoothTime = 0.10f;   // Seconds; larger values soften corrections more
const float wallFadeDistance = 15.0f; // Millimetres over which wall correction fades in/out
const float wallActiveDistance = 60.0f;

// Movement string and tracking
// char* moveSetChar = "fffrffrfrflflfrfrflfrflflffrrffrfrflfrflflfrfrflflfflfff";
// char* moveSetChar = "fffrffrfrflflfrfrflfrflflff"; // 4,3 start down
// char* moveSetChar = "flflfrffffrfrflflfrffrfrflfrflflfrfrflflfflfffrrfffrffrfrflflfrfrflfrflflfflfrfrflflfffflfrfrfrrflflfrffffrfrflflfrffrfrflfrflflfrfrflflfflfffrrfffrffrfrflflfrfrflfrflflfflfrfrflflfffflfrfrf";

// char* moveSetChar = "frfr";
// char* moveSetChar = "ffffrrffffllffffrrffffllffffllffffrrffff";
// char* moveSetChar = "frfflflfrfrffrflflfrr";
char* moveSetChar = "fffrflffflflfrfrflflfrfffrflflfrflfffrflflfrflffrrrrllllrlrlrl";
int currentStringIndex = 0;
float currentMoveDistance = 0.0f;
MoveType currentState = IDLE;
int wireTimeoutCount = 0;
float smoothedWallCorrection = 0.0f;
int previousLeftDistance = 255;
int previousRightDistance = 255;

// ====================== FUNCTION DECLARATIONS ======================
void updateLidars();
bool isFrontWallDetected();
bool isLeftWallDetected();
bool isRightWallDetected();
int applyRamping(int target, int previous);
int applyDeadzone(int pwm);
float smoothWallCorrection(float target, float dt);
void setMotorPWM(int leftPWM, int rightPWM);
void stopMotors();
bool settle(float dt);
bool forward(float setDist, float dt);
bool turn(float setAngle, float dt);
void processNextMove();
void initializeMove(MoveType moveType);
void moveFinished();

// ====================== SETUP ======================
void setup() {
  Wire.begin();
  Wire.setWireTimeout(5000, true);
  Serial.begin(9600);

  // Setup sensor pins
  for (int i = 0; i < 3; i++) {
    pinModeFast(sensorPins[i], OUTPUT);
    digitalWriteFast(sensorPins[i], LOW);
  }

  // Initialize sensors with different addresses
  VL6180X* sensors[] = {&sensorLeft, &sensorFront, &sensorRight};
  // byte lidar_addresses[] = {0x54, 0x56, 0x58};

  for (int i = 0; i < 3; i++) {
    digitalWriteFast(sensorPins[i], HIGH);
    delay(50);
    sensors[i]->init();
    sensors[i]->configureDefault();
    sensors[i]->setTimeout(250);
    sensors[i]->setAddress(lidar_addresses[i]);
    delay(100);
  }

  // Initialize IMU
  byte status = mpu.begin();
  while(status != 0) {}
  delay(1000);
  mpu.calcOffsets();
  delay(1000);
  
  referenceAngle = mpu.getAngleZ();
  startTime = millis();
  
  Serial.print("Move string length: ");
  Serial.println(strlen(moveSetChar));
}

// ====================== MAIN LOOP ======================
void loop() {
  mpu.update();
  updateLidars();

  if (Wire.getWireTimeoutFlag()) {
    wireTimeoutCount++;
    Wire.clearWireTimeoutFlag();
    Serial.print("Wire timeout: ");
    Serial.println(wireTimeoutCount);
  }

  static unsigned long lastTime = millis();
  unsigned long currTime = millis();
  float dt = (currTime - lastTime) / 1000.0f;
  lastTime = currTime;

  if (currentStringIndex >= strlen(moveSetChar) && currentState == IDLE) {
    stopMotors();
    return;
  }

  switch (currentState) {
    case IDLE:
      processNextMove();
      break;
    case SETTLING:
      if (settle(dt)) {
        Serial.println("Settling complete");
        stopMotors();
        referenceAngle = mpu.getAngleZ();
        currentState = IDLE;
      }
      break;
    case MOVE_FORWARD:
      if (isFrontWallDetected()) {
        Serial.println("Wall detected - settling!");
        pidStraight.reset();
        previousPWMForward = 0;
        currentState = SETTLING;
        break;
      }
      if (forward(currentMoveDistance, dt)) {
        moveFinished();
      }
      break;
    case TURN_LEFT:
    case TURN_RIGHT:
      if (turn(turnAngle, dt)) {
        moveFinished();
      }
      break;
  }
}

// ====================== SENSOR FUNCTIONS ======================
void updateLidars() {
  int newReading = 0;
  switch (currentSensor) {
    case 0: // Left
      newReading = sensorLeft.readRangeSingleMillimeters();
      if (!sensorLeft.timeoutOccurred()) distances[0] = newReading;
      break;
    case 1: // Front
      newReading = sensorFront.readRangeSingleMillimeters();
      if (!sensorFront.timeoutOccurred()) distances[1] = newReading;
      break;
    case 2: // Right
      newReading = sensorRight.readRangeSingleMillimeters();
      if (!sensorRight.timeoutOccurred()) distances[2] = newReading;
      break;
  }
  currentSensor = (currentSensor + 1) % 3;
}

bool isFrontWallDetected() { 
  return distances[1] < WALL_DISTANCE_THRESHOLD; 
}

bool isLeftWallDetected() { 
  return distances[0] < SIDE_WALL_DISTANCE_THRESHOLD; 
}

bool isRightWallDetected() { 
  return distances[2] < SIDE_WALL_DISTANCE_THRESHOLD; 
}

// ====================== HELPER FUNCTIONS ======================
int applyRamping(int target, int previous) {
  int delta = target - previous;
  if (abs(delta) > maxDeltaPWM) {
    delta = (delta > 0) ? maxDeltaPWM : -maxDeltaPWM;
  }
  return previous + delta;
}

int applyDeadzone(int pwm) {
  if (abs(pwm) < PWM_THRESHOLD && abs(pwm) > 0) {
    return map(abs(pwm), 0, PWM_THRESHOLD, PWM_MIN_SPEED, PWM_THRESHOLD) * (pwm > 0 ? 1 : -1);
  }
  return pwm;
}

float smoothWallCorrection(float target, float dt) {
  if (dt <= 0.0f) return smoothedWallCorrection;

  float alpha = dt / (wallSmoothTime + dt);
  smoothedWallCorrection += alpha * (target - smoothedWallCorrection);
  return smoothedWallCorrection;
}

void setMotorPWM(int leftPWM, int rightPWM) {
  pwmLeft = leftPWM;
  pwmRight = rightPWM;
  motorLeft.setPWM(-pwmLeft);
  motorRight.setPWM(pwmRight);
}

void stopMotors() {
  setMotorPWM(0, 0);
}

// ====================== MOVEMENT FUNCTIONS ======================
bool settle(float dt) {
  float frontError = wallTargetDist - distances[1];
  float distErr = pidStraight.update(0, frontError, dt);
  float yawErr = pidThetaStraight.update(referenceAngle, mpu.getAngleZ(), dt);

  // Wall following correction (same logic as forward function)
  float wallFollowCorrection = 0;
  bool leftWall = isLeftWallDetected();
  bool rightWall = isRightWallDetected();
  
  if (leftWall && rightWall) {
    float centeringError = ((SIDE_WALL_TARGET_DISTANCE - distances[0]) - 
                           (SIDE_WALL_TARGET_DISTANCE - distances[2])) / 2.0f;
    wallFollowCorrection = pidWallFollowLeft.update(0, centeringError, dt);
  } else if (leftWall) {
    wallFollowCorrection = pidWallFollowLeft.update(0, SIDE_WALL_TARGET_DISTANCE - distances[0], dt);
  } else if (rightWall) {
    wallFollowCorrection = -pidWallFollowRight.update(0, SIDE_WALL_TARGET_DISTANCE - distances[2], dt);
  }

  // Calculate PWM
  int basePWM = constrain(distErr, -PWM_MAX_SPEED * 0.8f, PWM_MAX_SPEED * 0.8f);
  int yawCorrection = constrain(yawErr * 0.3f, -50, 50);
  float filteredWallCorrection = smoothWallCorrection(wallFollowCorrection, dt);
  int wallCorrection = constrain(filteredWallCorrection * 0.25f, -40, 40);

  int targetLPWM = applyRamping(basePWM - yawCorrection, previousPWMForward);
  int targetRPWM = applyRamping(basePWM + yawCorrection, previousPWMForward);
  previousPWMForward = (targetLPWM + targetRPWM) / 2;

  setMotorPWM(applyDeadzone(targetLPWM - wallCorrection), 
             applyDeadzone(targetRPWM + wallCorrection));

  Serial.print("Settle: dist="); Serial.print(distances[1]);
  Serial.print(" err="); Serial.println(frontError);

  return fabs(frontError) < distDead;
}

bool forward(float setDist, float dt) {
  float currDistLeft = dualEncoder.getLeftRotation() * WHEEL_RADIUS;
  float currDistRight = dualEncoder.getRightRotation() * WHEEL_RADIUS;
  float avgDist = (-currDistLeft + currDistRight) / 2.0f;

  float distErr = pidStraight.update(setDist, avgDist, dt);
  float yawErr = pidThetaStraight.update(referenceAngle, mpu.getAngleZ(), dt);

  // Repeat the wall-following zones in every cell of a combined forward move.
  float wallFollowCorrection = 0;
  float cellPosition = fmod(avgDist, distanceF);
  if (cellPosition < 0.0f) cellPosition += distanceF;
  float distanceToCellEnd = distanceF - cellPosition;
  float distanceFromNearestEdge = min(cellPosition, distanceToCellEnd);

  if (distanceFromNearestEdge <= wallActiveDistance) {
    // Determine if we're in middle section (potential extrusion zone)
    bool inMiddleSection = distanceFromNearestEdge > 35.0f;
    
    // Use different thresholds based on section
    int leftThreshold = inMiddleSection ? 90 : SIDE_WALL_DISTANCE_THRESHOLD;
    int rightThreshold = inMiddleSection ? 90 : SIDE_WALL_DISTANCE_THRESHOLD;
    
    bool leftWall = (distances[0] < leftThreshold);
    bool rightWall = (distances[2] < rightThreshold);
    
    // Additional extrusion detection - sudden distance changes
    bool leftExtrusionDetected = false;
    bool rightExtrusionDetected = false;
    
    if (inMiddleSection) {
      // Detect sudden 8-15mm jump (extrusion detection)
      int leftDelta = abs(distances[0] - previousLeftDistance);
      int rightDelta = abs(distances[2] - previousRightDistance);
      
      if (leftDelta > 8 && leftDelta < 15 && distances[0] < previousLeftDistance) {
        leftExtrusionDetected = true;
      }
      if (rightDelta > 8 && rightDelta < 15 && distances[2] < previousRightDistance) {
        rightExtrusionDetected = true;
      }
    }
    
    // Apply wall following, but ignore extrusion readings
    if (leftWall && rightWall && !leftExtrusionDetected && !rightExtrusionDetected) {
      float centeringError = ((SIDE_WALL_TARGET_DISTANCE - distances[0]) - 
                             (SIDE_WALL_TARGET_DISTANCE - distances[2])) / 2.0f;
      wallFollowCorrection = pidWallFollowLeft.update(0, centeringError, dt);
    } else if (leftWall && !leftExtrusionDetected) {
      wallFollowCorrection = pidWallFollowLeft.update(0, SIDE_WALL_TARGET_DISTANCE - distances[0], dt);
    } else if (rightWall && !rightExtrusionDetected) {
      wallFollowCorrection = -pidWallFollowRight.update(0, SIDE_WALL_TARGET_DISTANCE - distances[2], dt);
    }
    
    // Reduce wall following strength in middle section
    if (inMiddleSection) {
      wallFollowCorrection *= 0.5f; // 50% strength in transition zones
    }

    // Fade to zero over the outer part of the active zone instead of switching abruptly.
    float fadeStart = wallActiveDistance - wallFadeDistance;
    float wallBlend = constrain((wallActiveDistance - distanceFromNearestEdge) /
                                wallFadeDistance, 0.0f, 1.0f);
    if (distanceFromNearestEdge <= fadeStart) wallBlend = 1.0f;
    wallFollowCorrection *= wallBlend;
  } else {
    pidWallFollowLeft.reset();
    pidWallFollowRight.reset();
  }

  // Keep comparisons current even while wall correction is outside its active zone.
  previousLeftDistance = distances[0];
  previousRightDistance = distances[2];

  // Calculate PWM
  int basePWM = constrain(distErr, -PWM_MAX_SPEED * 0.8f, PWM_MAX_SPEED * 0.8f);
  int yawCorrection = constrain(yawErr * 0.3f, -50, 50);
  float filteredWallCorrection = smoothWallCorrection(wallFollowCorrection, dt);
  int wallCorrection = constrain(filteredWallCorrection * 0.25f, -40, 40);

  int targetLPWM = applyRamping(basePWM - yawCorrection, previousPWMForward);
  int targetRPWM = applyRamping(basePWM + yawCorrection, previousPWMForward);
  previousPWMForward = (targetLPWM + targetRPWM) / 2;

  setMotorPWM(applyDeadzone(targetLPWM - wallCorrection), 
             applyDeadzone(targetRPWM + wallCorrection));

  return fabs(avgDist - setDist) < distDead;
}

bool turn(float setAngle, float dt) {
  float angleError = referenceAngle - mpu.getAngleZ();
  int targetPWM = -pidTheta.update(0, angleError, dt);
  targetPWM = constrain(targetPWM, -PWM_MAX_SPEED * 0.6f, PWM_MAX_SPEED * 0.6f);
  targetPWM = applyRamping(targetPWM, previousPWMTurn);
  previousPWMTurn = targetPWM;
  targetPWM = applyDeadzone(targetPWM);

  setMotorPWM(-targetPWM, targetPWM);
  return fabs(angleError) <= thetaDead;
}

// ====================== STATE MACHINE FUNCTIONS ======================
void processNextMove() {
  if (currentStringIndex >= strlen(moveSetChar)) return;
  
  char cmd = tolower(moveSetChar[currentStringIndex]);
  currentStringIndex++;
  
  if (cmd == 'f') {
    int forwardCount = 1;
    while (currentStringIndex < strlen(moveSetChar) &&
           tolower(moveSetChar[currentStringIndex]) == 'f') {
      forwardCount++;
      currentStringIndex++;
    }

    currentMoveDistance = distanceF * forwardCount;
    initializeMove(MOVE_FORWARD);
  } else if (cmd == 'l') {
    initializeMove(TURN_LEFT);
  } else if (cmd == 'r') {
    initializeMove(TURN_RIGHT);
  }
}

void initializeMove(MoveType moveType) {
  dualEncoder.resetEncoders();
  pidStraight.reset();
  pidTheta.reset();
  pidThetaStraight.reset();

  if (moveType == MOVE_FORWARD) {
    referenceAngle = mpu.getAngleZ();
    previousPWMForward = 0;
    smoothedWallCorrection = 0.0f;
    previousLeftDistance = distances[0];
    previousRightDistance = distances[2];
    pidWallFollowLeft.reset();
    pidWallFollowRight.reset();
  } else {
    float currentAngle = mpu.getAngleZ();
    referenceAngle = (moveType == TURN_LEFT) ? currentAngle + turnAngle : currentAngle - turnAngle;
    previousPWMTurn = 0;
  }

  currentState = moveType;
  startTime = millis();
}

void moveFinished() {
  stopMotors();
  currentState = IDLE;
}
