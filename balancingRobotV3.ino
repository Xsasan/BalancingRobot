#include "I2Cdev.h"
#include <digitalWriteFast.h>

// --------------------- START custom settings ---------------------
const float INITIAL_TARGET_ANGLE = 2.0;
const float TIPOVER_ANGLE_OFFSET = 45; // stop motors if bot has tipped over
const float MAX_FULL_STEPS_PER_SECOND = 900;
const float MAX_ACCELLERATION = 250.0;
const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT = 0.9992; // how much to use gyro value compared to acceleratometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START PID settings ---------------------
const float PID_ANGLE_MAX = 100;
const float PID_ANGLE_P_GAIN = 5;
const float PID_ANGLE_I_GAIN = 0.05;
const float PID_ANGLE_D_GAIN = -250;
const float PID_ANGLE_I_MAX = 20;
const float PID_ANGLE_D_MAX = 20;

const float PID_SPEED_MAX = 100;
const float PID_SPEED_P_GAIN = 7;
const float PID_SPEED_I_GAIN = 0.0;
const float PID_SPEED_D_GAIN = 0;
const float PID_SPEED_I_MAX = 100000;
const float PID_SPEED_D_MAX = 10000;

const float PID_POSITION_MAX = 100;
const float PID_POSITION_P_GAIN = -6;
const float PID_POSITION_I_GAIN = 0;
const float PID_POSITION_D_GAIN = 0;
const float PID_POSITION_I_MAX = 50000;
const float PID_POSITION_D_MAX = 100;
// --------------------- END PID settings ---------------------

// --------------------- START hardware settings ---------------------
const int MOTOR_STEPS_PER_360 = 200;
const int MICROSTEPPING = 32; // microstepping selected on the driver
const int MINIMUM_PIN_DELAY_MICROS = 2; // driver specific: how long a pin has to hold the output level to be registered by the driver (1.9us for DRV8834)
// ---------------------  END hardware settings  ---------------------

// --------------------- START wiring settings ---------------------
const int PIN_MOTOR_1_STEP = 13;
const int PIN_MOTOR_2_STEP = 12;
const int PIN_MOTOR_1_DIRECTION = 10;
const int PIN_MOTOR_2_DIRECTION = 9;
// ---------------------  END wiring settings  ---------------------

// --------------------- START calculated constants ---------------------
const int STEPS_PER_ROTATION = MOTOR_STEPS_PER_360*MICROSTEPPING;
const float STEPS_PER_DEGREE = STEPS_PER_ROTATION/360.0;
const int MAX_STEPS_PER_SECOND = MAX_FULL_STEPS_PER_SECOND*MICROSTEPPING;
const int MIN_STEP_FREQUENCY_MICROS = 1000000/MAX_STEPS_PER_SECOND;
// ---------------------  END calculated constants  ---------------------

// --------------------- START various constants ---------------------
const int MOTOR_LEFT_ID = 0;
const int MOTOR_RIGHT_ID = 1;
// ---------------------  END various constants  ---------------------

// --------------------- START gyro variables ---------------------
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
// ---------------------  END gyro variables  ---------------------

// --------------------- START motor variables ---------------------
boolean last_motor_direction[2];
unsigned long last_motor_step[2];
int last_motor_step_interval[2];
long motor_steps[2];
// ---------------------  END motor variables  ---------------------

// --------------------- START pitch calculation variables ---------------------
float pitch = INITIAL_TARGET_ANGLE;
float pitchChange;
float pitchAcc;
//float pid_angle_error;
float pid_angle_error_motor1;
float pid_angle_error_motor2;
unsigned long lastPitchCalculationTime;
unsigned long pitchCalculation_delta_t;
// ---------------------  END pitch calculation variables  ---------------------

// --------------------- START speed calculation variables ---------------------
float BODY_SPEED_COEFFICIENT = 0.02;
float bodySpeed;
//float pid_speed_error;
float pid_speed_error_motor1;
float pid_speed_error_motor2;
// ---------------------  END speed calculation variables  ---------------------

// --------------------- START position calculation variables ---------------------
//float pid_position_error;
float pid_position_error_motor1;
float pid_position_error_motor2;
unsigned long lastPositionCalculationTime;
unsigned long positionCalculation_delta_t;
// ---------------------  END position calculation variables  ---------------------

// --------------------- START step calculation variables ---------------------
//float partialSteps;
float partialSteps_motor1;
float partialSteps_motor2;
//float stepsPerSecond;
float stepsPerSecond_motor1;
float stepsPerSecond_motor2;
//float stepTimeMicros;
float stepTimeMicros_motor1;
float stepTimeMicros_motor2;
// ---------------------  END step calculation variables  ---------------------

// --------------------- START pid variables ---------------------
//float pid_angle_setpoint = INITIAL_TARGET_ANGLE;
float pid_angle_setpoint_motor1 = INITIAL_TARGET_ANGLE;
float pid_angle_setpoint_motor2 = INITIAL_TARGET_ANGLE;
//float pid_angle_output;
float pid_angle_output_motor1;
float pid_angle_output_motor2;
//float pid_angle_i;
float pid_angle_i_motor1;
float pid_angle_i_motor2;

//float pid_speed_setpoint = 0.0;
float pid_speed_setpoint_motor1 = 0.0;
float pid_speed_setpoint_motor2 = 0.0;
//float pid_speed_output;
float pid_speed_output_motor1;
float pid_speed_output_motor2;
//float pid_speed_i;
float pid_speed_i_motor1;
float pid_speed_i_motor2;

//float pid_position_setpoint = 0.0;
float pid_position_setpoint_motor1 = 0.0;
float pid_position_setpoint_motor2 = 0.0;
//float pid_position_output;
float pid_position_output_motor1;
float pid_position_output_motor2;
//float pid_position_i;
float pid_position_i_motor1;
float pid_position_i_motor2;
// ---------------------  END pid variables  ---------------------

boolean enable = false;

void setup() {
  Serial.begin(57600); // startup serial communication with given baud rate
  Serial.println("\nStartup...");
  readAndInitErrorRegister(); // log the cause of the last restart, i.e. power loss
  
  // set motor control pins to output mode
  pinMode(PIN_MOTOR_1_STEP, OUTPUT);
  pinMode(PIN_MOTOR_2_STEP, OUTPUT);
  pinMode(PIN_MOTOR_1_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_2_DIRECTION, OUTPUT);

  // initialize stepper by doing one step
  stepMotor(MOTOR_LEFT_ID,1);
  stepMotor(MOTOR_RIGHT_ID,1);

  setupMPU9250();

  waitForTargetAngle();
}

void waitForTargetAngle(){
  // wait until bot is reasonably near target angle so we can start  
  Serial.println("Wait until bot alignment is near target angle...");
  int correctAngleCount = 0;
  while(correctAngleCount < 50){
    getAccelGyroData();
    calculatePitch();
    if (abs(INITIAL_TARGET_ANGLE-pitch) < 1.0){
      correctAngleCount++;
    } else {
      correctAngleCount = 0;
    }
  }
  enable = true;
  Serial.println("done!");
}

void loop() {
    if (Serial.available() > 0){
      Serial.setTimeout(50); // we don't want to wait for the usual 1 second timeout as the bot would tipover
      delay(5);
      pid_position_setpoint_motor1 = Serial.parseInt();
      delay(5);
      pid_position_setpoint_motor2 = Serial.parseInt();
      delay(5);
      while(Serial.available() > 0) {
        volatile char character = Serial.read(); // clear buffer of remaining characters
      }
      Serial.print("understood: ");
      Serial.print(pid_position_setpoint_motor1);
      Serial.print(" ");
      Serial.println(pid_position_setpoint_motor2);
    }
    
    if (!enable){
      waitForTargetAngle();
    }
    getAccelGyroData();
    calculatePitch();
    
    if (abs(pitch-INITIAL_TARGET_ANGLE) > TIPOVER_ANGLE_OFFSET){
      enable = false;
      //stepsPerSecond = 0;
      stepsPerSecond_motor1 = 0;
      stepsPerSecond_motor2 = 0;
      motor_steps[0] = 0;
      motor_steps[1] = 0;
      //pid_angle_i = 0;
      pid_angle_i_motor1 = 0;
      pid_angle_i_motor2 = 0;
      //pid_speed_i = 0;
      pid_speed_i_motor1 = 0;
      pid_speed_i_motor2 = 0;
      //pid_position_i = 0;
      pid_position_i_motor1 = 0;
      pid_position_i_motor2 = 0;
      bodySpeed = 0;
      return;
    }

    //calculatePidPosition();
    pid_position_output_motor1 = calculatePidPosition(pid_position_setpoint_motor1, pid_position_i_motor1, pid_position_error_motor1, motor_steps[1]);
    pid_position_output_motor2 = calculatePidPosition(pid_position_setpoint_motor2, pid_position_i_motor2, pid_position_error_motor2, motor_steps[0]);
    calculatePidSpeedSetpoint();
    
    calculateBodySpeed();
    //calculatePidSpeed();
    pid_speed_output_motor1 = calculatePidSpeed(pid_speed_setpoint_motor1, pid_speed_i_motor1, pid_speed_error_motor1);
    pid_speed_output_motor2 = calculatePidSpeed(pid_speed_setpoint_motor2, pid_speed_i_motor2, pid_speed_error_motor2);
    
    calculatePidAngleSetpoint();
    //calculatePidAngle();
    pid_angle_output_motor1 = calculatePidAngle(pid_angle_setpoint_motor1, pid_angle_i_motor1, pid_angle_error_motor1);
    pid_angle_output_motor2 = calculatePidAngle(pid_angle_setpoint_motor2, pid_angle_i_motor2, pid_angle_error_motor2);

    //int stepCount = calculateStepCount();
    int stepCount_motor1 = calculateStepCount(pid_angle_output_motor1, stepsPerSecond_motor1, partialSteps_motor1, stepTimeMicros_motor1);
    int stepCount_motor2 = calculateStepCount(pid_angle_output_motor2, stepsPerSecond_motor2, partialSteps_motor2, stepTimeMicros_motor2);
    stepMotor(MOTOR_LEFT_ID, stepCount_motor1);
    stepMotor(MOTOR_RIGHT_ID, stepCount_motor2);
}

void calculatePitch() {
    // calulate time between two accel/gyro datasets
    unsigned long micros2 = micros();
    pitchCalculation_delta_t = micros2 - lastPitchCalculationTime;
    lastPitchCalculationTime = micros2;
    
    float squaresum = ay*ay+az*az;
    float delta_t_seconds = (float)pitchCalculation_delta_t*0.000001;
    pitchChange = gz*delta_t_seconds;
    pitch += pitchChange;
    pitchAcc = atan(ax/sqrt(squaresum))*RAD_TO_DEG;
    
    // use complementary filter to get accurate pitch fast and without drift
    pitch = COMPLEMENTARY_FILTER_GYRO_COEFFICIENT*pitch + (1.0-COMPLEMENTARY_FILTER_GYRO_COEFFICIENT)*pitchAcc;
}
  
float calculatePidAngle(float& pid_angle_setpoint, float& pid_angle_i, float& pid_angle_error){
  float lastError = pid_angle_error;
  pid_angle_error = pitch - pid_angle_setpoint;
  float error_change = pid_angle_error - lastError;
  float timeFactor = pitchCalculation_delta_t * 0.0001;

  // integrate error, but limit it to a reasonable value
  float pid_i_change = PID_ANGLE_I_GAIN * pid_angle_error * timeFactor;
  pid_angle_i = ensureRange(pid_angle_i+pid_i_change,-PID_ANGLE_I_MAX,PID_ANGLE_I_MAX);

  float pid_d = PID_ANGLE_D_GAIN * error_change * timeFactor;
  pid_d = ensureRange(pid_d,-PID_ANGLE_D_MAX,PID_ANGLE_D_MAX);
  
  float pid = PID_ANGLE_P_GAIN * pid_angle_error + pid_angle_i + pid_d;

  return ensureRange(pid,-PID_ANGLE_MAX,PID_ANGLE_MAX);
}

void calculateBodySpeed(){
  float delta_t_seconds = pitchCalculation_delta_t*0.001;
  float factor = pitchChange*STEPS_PER_DEGREE;
  float bodyStepsPerSecond = (stepsPerSecond_motor1+stepsPerSecond_motor2)/2-factor*gz;
  bodySpeed = bodySpeed*(1-BODY_SPEED_COEFFICIENT)+bodyStepsPerSecond*BODY_SPEED_COEFFICIENT;
  //Serial.println((String)stepsPerSecond + " " + (String)(factor*gz));
}

float calculatePidSpeed(float& pid_speed_setpoint, float& pid_speed_i, float& pid_speed_error){
  float lastError = pid_speed_error;
  pid_speed_error = bodySpeed - pid_speed_setpoint;
  float error_change = pid_speed_error - lastError;
  float timeFactor = pitchCalculation_delta_t * 0.0001;

  // integrate error, but limit it to a reasonable value
  float pid_i_change = PID_SPEED_I_GAIN * pid_speed_error * timeFactor;
  pid_speed_i = ensureRange(pid_speed_i+pid_i_change,-PID_SPEED_I_MAX,PID_SPEED_I_MAX);

  float pid_d = PID_SPEED_D_GAIN * error_change * timeFactor;
  pid_d = ensureRange(pid_d,-PID_SPEED_D_MAX,PID_SPEED_D_MAX);
  
  float pid = PID_SPEED_MAX/MAX_STEPS_PER_SECOND*0.1*(PID_SPEED_P_GAIN * pid_speed_error + pid_speed_i + pid_d);
  
  return ensureRange(pid,-PID_SPEED_MAX,PID_SPEED_MAX);
}

void calculatePidAngleSetpoint(){
  pid_angle_setpoint_motor1 = INITIAL_TARGET_ANGLE + pid_speed_output_motor1*(0.8/PID_SPEED_MAX*TIPOVER_ANGLE_OFFSET);
  pid_angle_setpoint_motor2 = INITIAL_TARGET_ANGLE + pid_speed_output_motor2*(0.8/PID_SPEED_MAX*TIPOVER_ANGLE_OFFSET);
}

float calculatePidPosition(float& pid_position_setpoint, float& pid_position_i, float& pid_position_error, long& motor_steps){
  float lastError = pid_position_error;
  pid_position_error = motor_steps - pid_position_setpoint;
  float error_change = pid_position_error - lastError;
  float timeFactor = pitchCalculation_delta_t * 0.0001;

  // integrate error, but limit it to a reasonable value
  float pid_i_change = PID_POSITION_I_GAIN * pid_position_error * timeFactor;
  pid_position_i = ensureRange(pid_position_i+pid_i_change,-PID_POSITION_I_MAX,PID_POSITION_I_MAX);

  float pid_d = PID_POSITION_D_GAIN * error_change * timeFactor;
  pid_d = ensureRange(pid_d,-PID_POSITION_D_MAX,PID_POSITION_D_MAX);
  
  float pid = PID_POSITION_MAX/MAX_STEPS_PER_SECOND*0.5*(PID_POSITION_P_GAIN * pid_position_error + pid_position_i + pid_d);
  
  return ensureRange(pid,-PID_POSITION_MAX,PID_POSITION_MAX);
}

void calculatePidSpeedSetpoint(){
  //pid_speed_setpoint = 0.0 + pid_position_output/PID_POSITION_MAX*MAX_STEPS_PER_SECOND*0.25;
  
  pid_speed_setpoint_motor1 = 0.0 + pid_position_output_motor1/PID_POSITION_MAX*MAX_STEPS_PER_SECOND*0.25;
  pid_speed_setpoint_motor2 = 0.0 + pid_position_output_motor2/PID_POSITION_MAX*MAX_STEPS_PER_SECOND*0.25;
  //Serial.println(pid_speed_setpoint);
}

int calculateStepCount(float& pid_angle_output, float& stepsPerSecond, float& partialSteps, float& stepTimeMicros){  
    float steps = howManySteps(pid_angle_output, stepsPerSecond, partialSteps, stepTimeMicros);
    int stepCount = (int)steps;
    partialSteps = steps - stepCount;

    return stepCount;
}

float howManySteps(float& pid_angle_output, float& stepsPerSecond, float& partialSteps, float& stepTimeMicros){
  unsigned long currentTime = micros();
  float factor = (float)pid_angle_output/PID_ANGLE_MAX;
  float lastStepsPerSecond = stepsPerSecond;
  stepsPerSecond = -factor * (float)MAX_STEPS_PER_SECOND;

  stepsPerSecond = ensureRange(stepsPerSecond, lastStepsPerSecond-MAX_ACCELLERATION, lastStepsPerSecond+MAX_ACCELLERATION);
  stepsPerSecond = ensureRange(stepsPerSecond, -MAX_STEPS_PER_SECOND, MAX_STEPS_PER_SECOND);
  
  stepTimeMicros = 1000000.0/stepsPerSecond;

  return (float)(currentTime-last_motor_step[MOTOR_LEFT_ID])/stepTimeMicros + partialSteps;
}

const void stepMotor(const int motorId, const int stepCount){
    long currentTime = micros();
    int direction = stepCount >= 0 ? HIGH : LOW;
    
    if (last_motor_direction[motorId] != direction){
      if (direction == LOW){
        switch (motorId){
          case 0:  digitalWriteFast(PIN_MOTOR_1_DIRECTION, LOW); break;
          case 1:  digitalWriteFast(PIN_MOTOR_2_DIRECTION, LOW); break;
        }
      } else {
        switch (motorId){
          case 0:  digitalWriteFast(PIN_MOTOR_1_DIRECTION, HIGH); break;
          case 1:  digitalWriteFast(PIN_MOTOR_2_DIRECTION, HIGH); break;
        }
      }
      last_motor_direction[motorId] = direction;
      delayMicroseconds(MINIMUM_PIN_DELAY_MICROS);
    }

    for (int i = 0; i < abs(stepCount); i++){
      delayMicroseconds(MINIMUM_PIN_DELAY_MICROS);
      switch (motorId){
        case 0:  digitalWriteFast(PIN_MOTOR_1_STEP, LOW); break;
        case 1:  digitalWriteFast(PIN_MOTOR_2_STEP, LOW); break;
      }
      delayMicroseconds(MINIMUM_PIN_DELAY_MICROS);
      switch (motorId){
        case 0:  digitalWriteFast(PIN_MOTOR_1_STEP, HIGH); break;
        case 1:  digitalWriteFast(PIN_MOTOR_2_STEP, HIGH); break;
      }
    }
    
    last_motor_step_interval[motorId] = currentTime-last_motor_step[motorId];
    last_motor_step[motorId] = currentTime;
    motor_steps[motorId] += stepCount;
}

//-------------------------------------------------------------------------
//--------------------------- helper methos -------------------------------
//-------------------------------------------------------------------------

int sign(float value){
  if (value > 0){
    return 1;
  } else if (value < 0){
    return -1;
  } else {
    return 0;
  }
}

float ensureRange(float value, float val1, float val2){
  float min = val1 < val2 ? val1 : val2;
  float max = val1 >= val2 ? val1 : val2;
  if (value < min){
    return min;
  } else if (value > max){
    return max;
  }
  return value;
}

// Check to see if we are recovering from a reset event and clear the error register
void readAndInitErrorRegister(){
  boolean mcusrMessageFound = false;
  if(MCUSR & WDRF) {
    Serial.println("Rebooting from a Watchdog Reset.\n");
    mcusrMessageFound=true;
  }
  if(MCUSR & BORF) {
    Serial.println("Rebooting from a Brown-out Reset.\n");
    mcusrMessageFound=true;
  }
  if(MCUSR & EXTRF) {
    Serial.println("Rebooting from an External Reset.\n");
    mcusrMessageFound=true;
  }
  if(MCUSR & PORF) {
    Serial.println("Rebooting from a Power Reset.\n");
    mcusrMessageFound=true;
  }

  if (!mcusrMessageFound){
    if (MCUSR == 0x00){
      Serial.println("Rebooting with emtpy MCUSR register");
    } else {
      Serial.println("Rebooting from an unknown reason : " + MCUSR);
    }
  }

  // Clear register
  MCUSR = 0x00;  
}



//-------------------------------------------------------------------------
//---------------------- MPU9250 specific stuff ---------------------------
//-------------------------------------------------------------------------

const int I2C_ADDRESS_GYRO = 0x68;
const int I2C_ADDRESS_MAGNETOMETER = 0x0C;

const float ACCERELOMETER_G_CONFIG = 2.0;
const float GYRO_DEG_PER_SECOND_CONFIG = 250.0;

const float GYRO_RANGE_FACTOR = GYRO_DEG_PER_SECOND_CONFIG/32768.0;
const float ACCEL_RANGE_FACTOR = ACCERELOMETER_G_CONFIG/32768.0;
const float COMPASS_RANGE_FACTOR = 4800.0/32768.0;

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

void getAccelGyroData(void){
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

void getCompassData(void){
  uint8_t buffer[6];
  I2Cdev::readBytes(0x0C, 0x03, 6, buffer);
  
  mx = (((int16_t)(buffer[1]) << 8) | buffer[0]) * COMPASS_RANGE_FACTOR;
  my = (((int16_t)(buffer[3]) << 8) | buffer[2]) * COMPASS_RANGE_FACTOR;
  mz = (((int16_t)(buffer[5]) << 8) | buffer[4]) * COMPASS_RANGE_FACTOR; 
}
