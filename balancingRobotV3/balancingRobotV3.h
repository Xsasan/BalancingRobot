#include "I2Cdev.h"
#include <digitalWriteFast.h>
#include "notes.h"
#include "util/delay.h"

#define I2C_ADDRESS_GYRO  0x68
#define MOTOR_LEFT_ID 0
#define MOTOR_RIGHT_ID 1
#define MOTOR_TOP_ID 2
#define PIN_MOTOR_1_STEP 13
#define PIN_MOTOR_2_STEP 12
#define PIN_MOTOR_3_STEP 11
#define PIN_MOTOR_1_DIRECTION 17
#define PIN_MOTOR_2_DIRECTION 16
#define PIN_MOTOR_3_DIRECTION 15
#define MINIMUM_PIN_DELAY_MICROS 1.9

// --------------------- START custom settings ---------------------
const float MAX_FULL_STEPS_PER_SECOND = 900;
const float MAX_BODY_SPEED_FACTOR_LIMIT = 0.45;
float max_body_speed_factor = MAX_BODY_SPEED_FACTOR_LIMIT/2.0;

extern const float INITIAL_TARGET_ANGLE;
extern const float STARTUP_ANGLE_TOLERANCE;
extern const float TIPOVER_ANGLE_OFFSET; // stop motors if bot has tipped over
extern const float MAX_ACCELLERATION;
const float MAX_ACCELLERATION_UNTIL_FULL_STEPS_PER_SECOND = MAX_FULL_STEPS_PER_SECOND/2.0;
extern const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT; // how much to use gyro value compared to accerelometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START hardware settings ---------------------
const int MOTOR_STEPS_PER_360 = 200;
const int MICROSTEPPING = 32; // microstepping selected on the driver
const float POWER_LIMIT_SETTLE_TIME_MICROS = 200; // how long it takes until changes on power limit take effect
// ---------------------  END hardware settings  ---------------------

// --------------------- START calculated constants ---------------------
const int STEPS_PER_ROTATION = MOTOR_STEPS_PER_360 * MICROSTEPPING;
const float STEPS_PER_DEGREE = STEPS_PER_ROTATION / 360.0;
const int MAX_STEPS_PER_SECOND = MAX_FULL_STEPS_PER_SECOND * MICROSTEPPING;
const int MIN_STEP_FREQUENCY_MICROS = 1000000 / MAX_STEPS_PER_SECOND;
// ---------------------  END calculated constants  ---------------------

// --------------------- START motor variables ---------------------
const int POWER_LIMIT_STARTUP_VALUE = 120;
int powerLimit = 0; // 0-255 where 255 = maximum power (about 0.6A depending on setting of potentiometer on DRV8834)
// ---------------------  END motor variables  ---------------------

// --------------------- START wiring settings ---------------------
const int PIN_BUZZER = 3;
const int PIN_POWER_LIMIT = 9;
// ---------------------  END wiring settings  ---------------------

// --------------------- START PID settings ---------------------
const float PID_ANGLE_MAX = 100;
extern const float PID_ANGLE_P_GAIN;
extern const float PID_ANGLE_I_GAIN;
extern const float PID_ANGLE_D_GAIN;
extern const float PID_ANGLE_I_MAX;
extern const float PID_ANGLE_D_MAX;

const float PID_SPEED_MAX = 100;
extern const float PID_SPEED_P_GAIN;
extern const float PID_SPEED_I_GAIN;
extern const float PID_SPEED_D_GAIN;
extern const float PID_SPEED_I_MAX;
extern const float PID_SPEED_D_MAX;

const float PID_POSITION_MAX = 100;
extern const float PID_POSITION_P_GAIN;
extern const float PID_POSITION_I_GAIN;
extern const float PID_POSITION_D_GAIN;
extern const float PID_POSITION_I_MAX;
extern const float PID_POSITION_D_MAX;
// --------------------- END PID settings ---------------------

// --------------------- START pid variables ---------------------
float pid_angle_setpoint_motor1;
float pid_angle_setpoint_motor2;
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

// --------------------- START accelgyro variables ---------------------
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
// ---------------------  END accelgyro variables  ---------------------

// --------------------- START pitch calculation variables ---------------------
float pitch;
float pitchChange;
float pitchAcc;
float pid_angle_error_motor1;
float pid_angle_error_motor2;
unsigned long lastPitchCalculationTime;
unsigned long pitchCalculation_delta_t;
// ---------------------  END pitch calculation variables  ---------------------

// --------------------- START speed calculation variables ---------------------
const float BODY_SPEED_COEFFICIENT = 0.02;
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

const int I2C_ADDRESS_MAGNETOMETER = 0x0C;
const float COMPASS_RANGE_FACTOR = 4800.0 / 32768.0;

const float RAMPUP_SPEED_COEFFICIENT = 0.0015;


float ensureRange(float value, float val1, float val2) {
  float min = val1 < val2 ? val1 : val2;
  float max = val1 >= val2 ? val1 : val2;
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  }
  return value;
}

float howManySteps(float pid_angle_output, float& stepsPerSecond, float& partialSteps, long motor_step_iteration_interval, float rotation_stepsPerSecond, float max_accelleration) {
  float factor = (float)pid_angle_output / PID_ANGLE_MAX;
  float lastStepsPerSecond = stepsPerSecond;
  stepsPerSecond = -factor * (float)MAX_STEPS_PER_SECOND + rotation_stepsPerSecond;

  stepsPerSecond = ensureRange(stepsPerSecond, lastStepsPerSecond - max_accelleration, lastStepsPerSecond + max_accelleration);
  stepsPerSecond = ensureRange(stepsPerSecond, -MAX_STEPS_PER_SECOND, MAX_STEPS_PER_SECOND);

  return (motor_step_iteration_interval * (stepsPerSecond * 0.000001)) + partialSteps;
}

int calculateStepCount(float pid_angle_output, float& stepsPerSecond, float& partialSteps, long motor_step_iteration_interval, float rotation_stepsPerSecond, float max_accelleration) {
  float steps = howManySteps(pid_angle_output, stepsPerSecond, partialSteps, motor_step_iteration_interval, rotation_stepsPerSecond, max_accelleration);
  int stepCount = (int)steps;  
  partialSteps = steps - stepCount;

  return stepCount;
}

void getCompassData(void) {
  uint8_t buffer[6];
  I2Cdev::readBytes(0x0C, 0x03, 6, buffer);

  mx = (((int16_t)(buffer[1]) << 8) | buffer[0]) * COMPASS_RANGE_FACTOR;
  my = (((int16_t)(buffer[3]) << 8) | buffer[2]) * COMPASS_RANGE_FACTOR;
  mz = (((int16_t)(buffer[5]) << 8) | buffer[4]) * COMPASS_RANGE_FACTOR;
}

float target_speed;
float target_rotation_speed;
float target_head_rotation_speed;

void stopSpeedRemoteControl(){
   target_speed = 0;
   target_rotation_speed = 0;
}

const int SPEED_MODE = 1;
const int POSITION_MODE = 2;

int mode = POSITION_MODE;
long lastDirectionInput;

float pid_speed_setpoint = 0.0;
float rotation_speed_setpoint = 0.0;
float head_rotation_speed_setpoint = 0.0;

void rampupDirectionControl(){
  pid_speed_setpoint = pid_speed_setpoint * (1-RAMPUP_SPEED_COEFFICIENT) + target_speed * RAMPUP_SPEED_COEFFICIENT;
  rotation_speed_setpoint  = rotation_speed_setpoint * (1-RAMPUP_SPEED_COEFFICIENT) + target_rotation_speed * RAMPUP_SPEED_COEFFICIENT;
  head_rotation_speed_setpoint  = head_rotation_speed_setpoint * (1-RAMPUP_SPEED_COEFFICIENT) + target_head_rotation_speed * RAMPUP_SPEED_COEFFICIENT;

  if (abs(target_speed - pid_speed_setpoint) < (0.02*MAX_STEPS_PER_SECOND*max_body_speed_factor)){
    pid_speed_setpoint = target_speed;
  }

  if (abs(target_rotation_speed - rotation_speed_setpoint) < (0.02*MAX_STEPS_PER_SECOND*max_body_speed_factor)){
    rotation_speed_setpoint = target_rotation_speed;
  }
  
  if (abs(target_head_rotation_speed - head_rotation_speed_setpoint) < (0.02*MAX_STEPS_PER_SECOND*max_body_speed_factor)){
    head_rotation_speed_setpoint = target_head_rotation_speed;
  }
}

void forward(){
	target_speed = 1; target_rotation_speed = 0;
}

void backward(){
	target_speed = -1; target_rotation_speed = 0;
}

void left(){
	target_speed = 0; target_rotation_speed = -0.5;
}

void head_left(){
	target_speed = 0; target_head_rotation_speed = -0.5;
}

void right(){
	target_speed = 0; target_rotation_speed = 0.5; 
}

void head_right(){
	target_speed = 0; target_head_rotation_speed = 0.5;
}

void forward_left(){
	target_speed = 1; target_rotation_speed = -0.25;
}

void forward_right(){
	target_speed = 1; target_rotation_speed = 0.25;
}

void backward_left(){
	target_speed = -1; target_rotation_speed = 0.25;
}

void backward_right(){
	target_speed = -1; target_rotation_speed = -0.25;
}

void stop(){
	target_speed = 0; target_rotation_speed = 0;
}

void head_stop(){
	target_speed = 0; target_head_rotation_speed = 0;
}

void speed(int value){
	max_body_speed_factor = value/10.0 * MAX_BODY_SPEED_FACTOR_LIMIT;
}

long lastWarningTone;
static int DEFAULT_WARNING_TONE_DURATION = 200;

void playWarningToneWithDuration(int frequency, int duration){
  if (millis()-lastWarningTone >  duration){
    tone(PIN_BUZZER, frequency, duration);
  }
}

void playWarningTone(int frequency){
  playWarningToneWithDuration(frequency, DEFAULT_WARNING_TONE_DURATION);
}

void serialReadDirection() {
  if (Serial.available() > 0) {
     lastDirectionInput = millis();
     mode = SPEED_MODE;
  
     char c = Serial.read();
     boolean validInput = true;

     switch (c){
      case 'f': forward(); break;
      case 'F': forward(); break;
      case 'b': backward(); break;
      case 'B': backward(); break;
      case 'l': left(); break;
      case 'L': left(); break;
      case 'r': right(); break;
      case 'R': right(); break;
      case 'G': forward_left(); break;
      case 'I': forward_right(); break;
      case 'H': backward_left(); break;
      case 'J': backward_right(); break;
      case 'V': playWarningToneWithDuration(150,750); break;
      case 'v': /* Horn off */ break;
      case 'X': /* extra on */ break;
      case 'x': /* extra off */ break;
      case 'W': head_left(); break;
      case 'w': head_stop(); break;
      case 'U': head_right(); break;
      case 'u': head_stop(); break;
      case 'D': stop(); break;
      case 'S': stop(); break;
      case '0': stop(); break;
      case '1': speed(1); break;
      case '2': speed(2); break;
      case '3': speed(3); break;
      case '4': speed(4); break;
      case '5': speed(5); break;
      case '6': speed(6); break;
      case '7': speed(7); break;
      case '8': speed(8); break;
      case '9': speed(9); break;
      case 'q': speed(10); break;
      default: validInput = false;
     }

     if (validInput){
        target_speed *= MAX_STEPS_PER_SECOND * max_body_speed_factor;
        target_rotation_speed *= MAX_STEPS_PER_SECOND * max_body_speed_factor;
        target_head_rotation_speed *= MAX_STEPS_PER_SECOND * max_body_speed_factor;
     }
  } else {
    if (millis() - lastDirectionInput > 2000){
      stopSpeedRemoteControl();
    }
  }

  if (mode == SPEED_MODE){
    rampupDirectionControl();
  }
}

float pid_position_setpoint_motor1 = 0.0;
float pid_position_setpoint_motor2 = 0.0;

void serialReadPosition() {
  if (Serial.available() > 0) {
    Serial.setTimeout(50); // we don't want to wait for the usual 1 second timeout as the bot would tipover
    delay(5);
    pid_position_setpoint_motor1 = Serial.parseInt();
    delay(5);
    pid_position_setpoint_motor2 = Serial.parseInt();
    delay(5);
    while (Serial.available() > 0) {
      volatile char character = Serial.read(); // clear buffer of remaining characters
    }
    Serial.print("understood: ");
    Serial.print(pid_position_setpoint_motor1);
    Serial.print(" ");
    Serial.println(pid_position_setpoint_motor2);
  }
}

// Beware: digitalWriteFast needs hardcoded constants to work! So all combinations be hardcoded like this
boolean last_motor_direction[3];
unsigned long last_motor_step_time; // the last time stepMotors() was called

float motor_step_iteration_interval; // how long between two calls of stepMotors()

long motor_steps[3];

const void stepMotors(int stepsMotor1, int stepsMotor2, int stepsMotor3){  
  int directionMotor1 = stepsMotor1 >= 0 ? HIGH : LOW;
  int directionMotor2 = stepsMotor2 >= 0 ? HIGH : LOW;
  int directionMotor3 = stepsMotor3 >= 0 ? HIGH : LOW;

  if (last_motor_direction[MOTOR_LEFT_ID] != directionMotor1) {
    if (directionMotor1 == LOW) {
        digitalWriteFast(PIN_MOTOR_1_DIRECTION, LOW);
    } else {
        digitalWriteFast(PIN_MOTOR_1_DIRECTION, HIGH);
    }
    last_motor_direction[MOTOR_LEFT_ID] = directionMotor1;
    _delay_us(MINIMUM_PIN_DELAY_MICROS);
  }
  if (last_motor_direction[MOTOR_RIGHT_ID] != directionMotor2) {
    if (directionMotor2 == LOW) {
        digitalWriteFast(PIN_MOTOR_2_DIRECTION, LOW);
    } else {
        digitalWriteFast(PIN_MOTOR_2_DIRECTION, HIGH);
    }
    last_motor_direction[MOTOR_RIGHT_ID] = directionMotor2;
    _delay_us(MINIMUM_PIN_DELAY_MICROS);
  }
  if (last_motor_direction[MOTOR_TOP_ID] != directionMotor3) {
    if (directionMotor3 == LOW) {
        digitalWriteFast(PIN_MOTOR_3_DIRECTION, LOW);
    } else {
        digitalWriteFast(PIN_MOTOR_3_DIRECTION, HIGH);
    }
    last_motor_direction[MOTOR_TOP_ID] = directionMotor3;
    _delay_us(MINIMUM_PIN_DELAY_MICROS);
  }

  int counterMaximum = max(max(abs(stepsMotor1),abs(stepsMotor2)),abs(stepsMotor3));
  for (int i = 0; i < counterMaximum; i++) {
    if (i < abs(stepsMotor1)){
      digitalWriteFast(PIN_MOTOR_1_STEP, LOW);
    }
    if (i < abs(stepsMotor2)){
      digitalWriteFast(PIN_MOTOR_2_STEP, LOW);
    }
    if (i < abs(stepsMotor3)){
      digitalWriteFast(PIN_MOTOR_3_STEP, LOW);
    }
    
    _delay_us(MINIMUM_PIN_DELAY_MICROS);
    
    if (i < abs(stepsMotor1)){
      digitalWriteFast(PIN_MOTOR_1_STEP, HIGH);
    }
    if (i < abs(stepsMotor2)){
      digitalWriteFast(PIN_MOTOR_2_STEP, HIGH);
    }
    if (i < abs(stepsMotor3)){
      digitalWriteFast(PIN_MOTOR_3_STEP, HIGH);
    }
    
    _delay_us(MINIMUM_PIN_DELAY_MICROS);
  }

  long currentTime = micros();
  if (currentTime - last_motor_step_time > 10000 || motor_step_iteration_interval == 0){
    motor_step_iteration_interval = 1000; // just a guess
  } else {
    motor_step_iteration_interval = motor_step_iteration_interval * 0.98 + (currentTime-last_motor_step_time) * 0.02; // smooth/average the iteration time
  }
  last_motor_step_time = currentTime;
  motor_steps[MOTOR_LEFT_ID] += stepsMotor1;
  motor_steps[MOTOR_RIGHT_ID] += stepsMotor2;
  motor_steps[MOTOR_TOP_ID] += stepsMotor3;
}

void setPowerLimit(){
  analogWrite(PIN_POWER_LIMIT,powerLimit);
}

// Check to see if we are recovering from a reset event and clear the error register
void readAndInitErrorRegister() {
  boolean mcusrMessageFound = false;
  if (MCUSR & WDRF) {
    Serial.println("Rebooting from a Watchdog Reset.\n");
    mcusrMessageFound = true;
  }
  if (MCUSR & BORF) {
    Serial.println("Rebooting from a Brown-out Reset.\n");
    mcusrMessageFound = true;
  }
  if (MCUSR & EXTRF) {
    Serial.println("Rebooting from an External Reset.\n");
    mcusrMessageFound = true;
  }
  if (MCUSR & PORF) {
    Serial.println("Rebooting from a Power Reset.\n");
    mcusrMessageFound = true;
  }

  if (!mcusrMessageFound) {
    if (MCUSR == 0x00) {
      Serial.println("Rebooting with emtpy MCUSR register");
    } else {
      Serial.println("Rebooting from an unknown reason : " + MCUSR);
    }
  }

  // Clear register
  MCUSR = 0x00;
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
  pid_speed_setpoint = 0.0 + (pid_position_output_motor1+pid_position_output_motor2)/2 / PID_POSITION_MAX * MAX_STEPS_PER_SECOND * max_body_speed_factor;
  rotation_speed_setpoint = 0.0 + (pid_position_output_motor1-pid_position_output_motor2) / PID_POSITION_MAX * MAX_STEPS_PER_SECOND * max_body_speed_factor;
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

void setupMPU9250() {
  // init I2C bus
  Wire.begin();
  Wire.setClock(400000); // "fast mode", default is 100000

  // initialize device
  Serial.print("Initializing MPU9250...");
  delay(500);
  // set clock source to gyro y-axis pll signal instead of internal 8 MHZ oscillator for better quality
  // this will disable low-power capabilitites but we don't use them anyway
  I2Cdev::writeBits(I2C_ADDRESS_GYRO, 0x6B, 2, 2, 0x02);
  // set gyro configruation to 250 deg/s
  I2Cdev::writeBits(I2C_ADDRESS_GYRO, 0x1B, 4, 2, 0x00);
  // set gyro configruation to 2g (g = earth gravitation (9.81m/s^2))
  I2Cdev::writeBits(I2C_ADDRESS_GYRO, 0x1C, 4, 2, 0x00);
  // enable low-pass filter to 44hz to reduce influence of vibrations
  I2Cdev::writeBits(I2C_ADDRESS_GYRO, 0x1A, 2, 3, 0x03);
  // enable magnetometer
  I2Cdev::writeByte(I2C_ADDRESS_MAGNETOMETER, 0x0A, 0x01);
  // disable MPU9250 sleep-mode
  I2Cdev::writeBit(I2C_ADDRESS_GYRO, 0x6B, 6, false);


  delay(500);
  Serial.println("done!");
}

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
