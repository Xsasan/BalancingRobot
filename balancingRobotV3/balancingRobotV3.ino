#include "I2Cdev.h"
#include <digitalWriteFast.h>
#include "notes.h"
#include "util/delay.h"
#include "botlib.h"

// --------------------- START custom settings ---------------------
const float INITIAL_TARGET_ANGLE = 2.5;
const float STARTUP_ANGLE_TOLERANCE = 3.0;
const float TIPOVER_ANGLE_OFFSET = 35; // stop motors if bot has tipped over
const float MAX_ACCELLERATION = 150.0;
const float MAX_ACCELLERATION_UNTIL_FULL_STEPS_PER_SECOND = 450;
const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT = 0.999; // how much to use gyro value compared to accerelometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START PID settings ---------------------
const float PID_ANGLE_P_GAIN = 5;
const float PID_ANGLE_I_GAIN = 0.05;
const float PID_ANGLE_D_GAIN = -250;
const float PID_ANGLE_I_MAX = 20;
const float PID_ANGLE_D_MAX = 20;

const float PID_SPEED_MAX = 100;
const float PID_SPEED_P_GAIN = 11.0;
const float PID_SPEED_I_GAIN = 0.01;
const float PID_SPEED_D_GAIN = 0;
const float PID_SPEED_I_MAX = 50000;
const float PID_SPEED_D_MAX = 10000;

const float PID_POSITION_MAX = 100;
const float PID_POSITION_P_GAIN = -5;
const float PID_POSITION_I_GAIN = 0;
const float PID_POSITION_D_GAIN = 0;
const float PID_POSITION_I_MAX = 20;
const float PID_POSITION_D_MAX = 20;
// --------------------- END PID settings ---------------------

// --------------------- START gyro variables ---------------------
float ax, ay, az;
float gx, gy, gz;
// ---------------------  END gyro variables  ---------------------

// --------------------- START pitch calculation variables ---------------------
float pitch = INITIAL_TARGET_ANGLE;
float pitchChange;
float pitchAcc;
float pid_angle_error_motor1;
float pid_angle_error_motor2;
unsigned long lastPitchCalculationTime;
unsigned long pitchCalculation_delta_t;
// ---------------------  END pitch calculation variables  ---------------------

// --------------------- START speed calculation variables ---------------------
float BODY_SPEED_COEFFICIENT = 0.02;
float bodySpeed;
float pid_speed_error_motor1;
float pid_speed_error_motor2;
// ---------------------  END speed calculation variables  ---------------------

// --------------------- START position calculation variables ---------------------
float pid_position_error_motor1;
float pid_position_error_motor2;
unsigned long lastPositionCalculationTime;
unsigned long positionCalculation_delta_t;
// ---------------------  END position calculation variables  ---------------------

// --------------------- START step calculation variables ---------------------
float partialSteps_motor1;
float partialSteps_motor2;
float partialSteps_motor3;
float stepsPerSecond_motor1;
float stepsPerSecond_motor2;
float stepsPerSecond_motor3;
// ---------------------  END step calculation variables  ---------------------

// --------------------- START pid variables ---------------------
float pid_angle_setpoint_motor1 = INITIAL_TARGET_ANGLE;
float pid_angle_setpoint_motor2 = INITIAL_TARGET_ANGLE;
float pid_angle_output_motor1;
float pid_angle_output_motor2;
float pid_angle_i_motor1;
float pid_angle_i_motor2;

float pid_speed_output_motor1;
float pid_speed_output_motor2;
float pid_speed_i_motor1;
float pid_speed_i_motor2;

float pid_position_output_motor1;
float pid_position_output_motor2;
float pid_position_i_motor1;
float pid_position_i_motor2;
// ---------------------  END pid variables  ---------------------

// ------------------------ START music ------------------------
// notes in the melody:
int melody[] = {
  NOTE_C4, 250, 30,
  NOTE_G3, 125, 30,
  NOTE_G3, 125, 30,
  NOTE_A3, 250, 30,
  NOTE_G3, 250, 300,
  NOTE_B3, 250, 30,
  NOTE_C4, 250, 30
};
float melodySpeedSlowdown = 1.1;
// ------------------------  END music  ------------------------

boolean enable = false;
long disableTime = 0;

void debugLoop(){  
    //Serial.println(head_rotation_speed_setpoint);
}

void setup() {
  Serial.begin(57600); // startup serial communication with given baud rate
  Serial.println("\nStartup...");
  readAndInitErrorRegister(); // log the cause of the last restart, i.e. power loss

  // set motor control pins to output mode
  pinMode(PIN_MOTOR_1_STEP, OUTPUT);
  pinMode(PIN_MOTOR_2_STEP, OUTPUT);
  pinMode(PIN_MOTOR_3_STEP, OUTPUT);
  pinMode(PIN_MOTOR_1_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_2_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_3_DIRECTION, OUTPUT);

  // set buzzer and power limit pin to output mode
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_POWER_LIMIT, OUTPUT);

  // set PWM on Pins 9 and 10 to 31250 Hz instead of default 488 Hz
  // for better reaction of power target (only works when power target is controlled with one of these pins)
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  
  setPowerLimit();
  _delay_us(POWER_LIMIT_SETTLE_TIME_MICROS); // wait for power limit to stabilize (capacitor chargeup)

  // initialize stepper by doing one step
  stepMotors(MICROSTEPPING, MICROSTEPPING, MICROSTEPPING);

  setupMPU9250();

  startupMelody();

  waitForTargetAngle();
}

void startupMelody() {
  for (int thisNote = 0; thisNote < sizeof(melody) / sizeof(int); thisNote += 3) {
    tone(PIN_BUZZER, melody[thisNote], melody[thisNote + 1]);

    delay(melodySpeedSlowdown * (melody[thisNote + 1] + melody[thisNote + 2]));
  }
  noTone(PIN_BUZZER);
}

void waitForTargetAngle() {
  // wait until bot is reasonably near target angle so we can start
  Serial.println("Wait until bot alignment is near target angle...");
  int correctAngleCount = 0;
  while (correctAngleCount < 10) {
    delay(50);
    getAccelGyroData();
    calculatePitchWithGyroCoefficient(0);
    Serial.println(pitch);
    float angleOffset = abs(INITIAL_TARGET_ANGLE - pitch);
    if (angleOffset < 15){
      float y = 15.0;
      playWarningToneWithDuration((y-min(angleOffset,y))/y*500,50);
    }
    if (angleOffset < STARTUP_ANGLE_TOLERANCE) {
      correctAngleCount++;      
    } else {
      correctAngleCount = 0;
    }
  }
  enableBot();
  Serial.println("done!");
}

boolean getAccelGyro = true;
int count = 0;
void loop() {
  if (count++ >= 50){
    count = 0;
    debugLoop();
  }
  //serialReadPosition();
  serialReadDirection();
  
  setPowerLimit();

  if (!enable) {
    waitForTargetAngle();
  }
  getAccelGyro = !getAccelGyro;
  if (getAccelGyro){
    getAccelGyroData();
    calculatePitch();

    if (abs(pitch - INITIAL_TARGET_ANGLE) > TIPOVER_ANGLE_OFFSET) {
      disableBot();
      return;
    }
  } else {
    if (mode >= POSITION_MODE){
      pid_position_output_motor1 = calculatePidPosition(pid_position_setpoint_motor1, pid_position_i_motor1, pid_position_error_motor1, motor_steps[MOTOR_RIGHT_ID]);
      pid_position_output_motor2 = calculatePidPosition(pid_position_setpoint_motor2, pid_position_i_motor2, pid_position_error_motor2, motor_steps[MOTOR_LEFT_ID]);
      calculatePidSpeedSetpoint();
    }
  
    calculateBodySpeed();
    pid_speed_output_motor1 = calculatePidSpeed(pid_speed_setpoint, pid_speed_i_motor1, pid_speed_error_motor1);
    pid_speed_output_motor2 = calculatePidSpeed(pid_speed_setpoint, pid_speed_i_motor2, pid_speed_error_motor2);
  
    calculatePidAngleSetpoint();
    pid_angle_output_motor1 = calculatePidAngle(pid_angle_setpoint_motor1, pid_angle_i_motor1, pid_angle_error_motor1);
    pid_angle_output_motor2 = calculatePidAngle(pid_angle_setpoint_motor2, pid_angle_i_motor2, pid_angle_error_motor2);
  }

  int stepCount_motor1 = calculateStepCount(pid_angle_output_motor1, stepsPerSecond_motor1, partialSteps_motor1, motor_step_iteration_interval, +rotation_speed_setpoint/2.0, MAX_ACCELLERATION);
  int stepCount_motor2 = calculateStepCount(pid_angle_output_motor2, stepsPerSecond_motor2, partialSteps_motor2, motor_step_iteration_interval, -rotation_speed_setpoint/2.0, MAX_ACCELLERATION);
  int stepCount_motor3 = head_rotation_speed_setpoint*5.0;
  
  /*if (abs(stepCount_motor1) > 2*MICROSTEPPING || abs(stepCount_motor2) > 2*MICROSTEPPING){
    playWarningTone(400);
  } else if (abs(stepCount_motor1) > 1.5*MICROSTEPPING || abs(stepCount_motor2) > 1.5*MICROSTEPPING){
    playWarningTone(300);
  } else if (abs(stepCount_motor1) > MICROSTEPPING || abs(stepCount_motor2) > MICROSTEPPING){
    playWarningTone(200);
  }*/

  stepMotors(stepCount_motor1,stepCount_motor2,stepCount_motor3);
}

void disableBot(){
  Serial.println("disabling bot");
  playWarningToneWithDuration(4000,750);
  disableTime = millis();
  
  enable = false;
  stepsPerSecond_motor1 = 0;
  stepsPerSecond_motor2 = 0;
  motor_steps[0] = 0;
  motor_steps[1] = 0;
  pid_angle_i_motor1 = 0;
  pid_angle_i_motor2 = 0;
  pid_speed_i_motor1 = 0;
  pid_speed_i_motor2 = 0;
  pid_position_i_motor1 = 0;
  pid_position_i_motor2 = 0;
  bodySpeed = 0;
      
  pid_angle_setpoint_motor1 = 0;
  pid_angle_setpoint_motor2 = 0;
  pid_speed_setpoint = 0;
  rotation_speed_setpoint = 0;
  pid_position_setpoint_motor1 = 0;
  pid_position_setpoint_motor2 = 0;
  
  powerLimit = 0;
}

void enableBot(){  
  enable = true;
  powerLimit = POWER_LIMIT_STARTUP_VALUE;
  _delay_us(POWER_LIMIT_SETTLE_TIME_MICROS); 
}

void calculatePitch() {
  calculatePitchWithGyroCoefficient(COMPLEMENTARY_FILTER_GYRO_COEFFICIENT);
}

void calculatePitchWithGyroCoefficient(float gyroCoefficient) {
  // calulate time between two accel/gyro datasets
  unsigned long micros2 = micros();
  pitchCalculation_delta_t = micros2 - lastPitchCalculationTime;
  lastPitchCalculationTime = micros2;

  float squaresum = ay * ay + az * az;
  float delta_t_seconds = (float)pitchCalculation_delta_t * 0.000001;
  pitchChange = gz * delta_t_seconds;
  pitch += pitchChange;
  pitchAcc = atan(ax / sqrt(squaresum)) * RAD_TO_DEG;

  // use complementary filter to get accurate pitch fast and without drift
  pitch = gyroCoefficient * pitch + (1.0 - gyroCoefficient) * pitchAcc;
}

float calculatePidAngle(float& pid_angle_setpoint, float& pid_angle_i, float& pid_angle_error) {
  float lastError = pid_angle_error;
  pid_angle_error = pitch - pid_angle_setpoint;
  float error_change = pid_angle_error - lastError;
  float timeFactor = pitchCalculation_delta_t * 0.001 * 0.1;

  // integrate error, but limit it to a reasonable value
  float pid_i_change = PID_ANGLE_I_GAIN * pid_angle_error * timeFactor;
  pid_angle_i = ensureRange(pid_angle_i + pid_i_change, -PID_ANGLE_I_MAX, PID_ANGLE_I_MAX);

  float pid_d = PID_ANGLE_D_GAIN * error_change * timeFactor;
  pid_d = ensureRange(pid_d, -PID_ANGLE_D_MAX, PID_ANGLE_D_MAX);

  float pid = PID_ANGLE_P_GAIN * pid_angle_error + pid_angle_i + pid_d;

  return ensureRange(pid, -PID_ANGLE_MAX, PID_ANGLE_MAX);
}

void calculateBodySpeed() {
  float delta_t_seconds = pitchCalculation_delta_t * 0.001 * 0.1;
  float factor = pitchChange * STEPS_PER_DEGREE;
  float bodyStepsPerSecond = (stepsPerSecond_motor1+stepsPerSecond_motor2)/2.0 - factor * gz;
  bodySpeed = bodySpeed * (1 - BODY_SPEED_COEFFICIENT) + bodyStepsPerSecond * BODY_SPEED_COEFFICIENT;
}

float calculatePidSpeed(float pid_speed_setpoint, float& pid_speed_i, float& pid_speed_error) {
  float lastError = pid_speed_error;
  pid_speed_error = bodySpeed - pid_speed_setpoint;
  float error_change = pid_speed_error - lastError;
  float timeFactor = pitchCalculation_delta_t * 0.001;

  // integrate error, but limit it to a reasonable value
  float pid_i_change = PID_SPEED_I_GAIN * pid_speed_error * timeFactor;
  pid_speed_i = ensureRange(pid_speed_i + pid_i_change, -PID_SPEED_I_MAX, PID_SPEED_I_MAX);

  float pid_d = PID_SPEED_D_GAIN * error_change * timeFactor;
  pid_d = ensureRange(pid_d, -PID_SPEED_D_MAX, PID_SPEED_D_MAX);

  float pid = PID_SPEED_MAX / MAX_STEPS_PER_SECOND * 0.1 * (PID_SPEED_P_GAIN * pid_speed_error + pid_speed_i + pid_d);

  return ensureRange(pid, -PID_SPEED_MAX, PID_SPEED_MAX);
}

void calculatePidAngleSetpoint() {
  pid_angle_setpoint_motor1 = INITIAL_TARGET_ANGLE + pid_speed_output_motor1 * (0.8 / PID_SPEED_MAX * TIPOVER_ANGLE_OFFSET);
  pid_angle_setpoint_motor2 = INITIAL_TARGET_ANGLE + pid_speed_output_motor2 * (0.8 / PID_SPEED_MAX * TIPOVER_ANGLE_OFFSET);
}

float calculatePidPosition(float& pid_position_setpoint, float& pid_position_i, float& pid_position_error, long& motor_steps) {
  float lastError = pid_position_error;
  pid_position_error = motor_steps - pid_position_setpoint;
  float error_change = pid_position_error - lastError;
  float timeFactor = pitchCalculation_delta_t * 0.001 * 0.1;

  // integrate error, but limit it to a reasonable value
  float pid_i_change = PID_POSITION_I_GAIN * pid_position_error * timeFactor;
  pid_position_i = ensureRange(pid_position_i + pid_i_change, -PID_POSITION_I_MAX, PID_POSITION_I_MAX);

  float pid_d = PID_POSITION_D_GAIN * error_change * timeFactor;
  pid_d = ensureRange(pid_d, -PID_POSITION_D_MAX, PID_POSITION_D_MAX);

  float pid = PID_POSITION_MAX / MAX_STEPS_PER_SECOND * 0.5 * (PID_POSITION_P_GAIN * pid_position_error + pid_position_i + pid_d);

  return ensureRange(pid, -PID_POSITION_MAX, PID_POSITION_MAX);
}

void calculatePidSpeedSetpoint() {  
  pid_speed_setpoint = 0.0 + (pid_position_output_motor1+pid_position_output_motor2)/2 / PID_POSITION_MAX * MAX_STEPS_PER_SECOND * MAX_BODY_SPEED_FACTOR;
  rotation_speed_setpoint = 0.0 + (pid_position_output_motor1-pid_position_output_motor2) / PID_POSITION_MAX * MAX_STEPS_PER_SECOND * MAX_BODY_SPEED_FACTOR;
}


//-------------------------------------------------------------------------
//--------------------------- helper methos -------------------------------
//-------------------------------------------------------------------------

int sign(float value) {
  if (value > 0) {
    return 1;
  } else if (value < 0) {
    return -1;
  } else {
    return 0;
  }
}

//-------------------------------------------------------------------------
//---------------------- MPU9250 specific stuff ---------------------------
//-------------------------------------------------------------------------
const float ACCERELOMETER_G_CONFIG = 2.0;
const float GYRO_DEG_PER_SECOND_CONFIG = 250.0;

const float GYRO_RANGE_FACTOR = GYRO_DEG_PER_SECOND_CONFIG / 32768.0;
const float ACCEL_RANGE_FACTOR = ACCERELOMETER_G_CONFIG / 32768.0;

void getAccelGyroData(void) {
  uint8_t buffer[14];
  I2Cdev::readBytes(I2C_ADDRESS_GYRO, 0x3B, 14, buffer);
  ax = ((((int16_t)buffer[0]) << 8) | buffer[1]) * ACCEL_RANGE_FACTOR;
  ay = ((((int16_t)buffer[2]) << 8) | buffer[3]) * ACCEL_RANGE_FACTOR;
  az = ((((int16_t)buffer[4]) << 8) | buffer[5]) * ACCEL_RANGE_FACTOR;
  // we don't need temperature, so bits 7 and 8 are ignored
  gx = ((((int16_t)buffer[8]) << 8) | buffer[9]) * GYRO_RANGE_FACTOR;
  gy = ((((int16_t)buffer[10]) << 8) | buffer[11]) * GYRO_RANGE_FACTOR;
  gz = ((((int16_t)buffer[12]) << 8) | buffer[13]) * GYRO_RANGE_FACTOR;
}
