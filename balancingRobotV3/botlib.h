#define I2C_ADDRESS_GYRO  0x68
#define MOTOR_LEFT_ID 0
#define MOTOR_RIGHT_ID 1
#define PIN_MOTOR_1_STEP 13
#define PIN_MOTOR_2_STEP 12
#define PIN_MOTOR_3_STEP 11
#define PIN_MOTOR_1_DIRECTION 17
#define PIN_MOTOR_2_DIRECTION 16
#define PIN_MOTOR_3_DIRECTION 15
#define MINIMUM_PIN_DELAY_MICROS 1.9

// ---------------------  START custom settings  ---------------------
const float MAX_FULL_STEPS_PER_SECOND = 900;
const float MAX_BODY_SPEED_FACTOR = 0.4;
// ---------------------  END custom settings  -----------------------

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
// ---------------------- END PID settings ----------------------

const int I2C_ADDRESS_MAGNETOMETER = 0x0C;
const float COMPASS_RANGE_FACTOR = 4800.0 / 32768.0;


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
  float mx, my, mz;
  uint8_t buffer[6];
  I2Cdev::readBytes(0x0C, 0x03, 6, buffer);

  mx = (((int16_t)(buffer[1]) << 8) | buffer[0]) * COMPASS_RANGE_FACTOR;
  my = (((int16_t)(buffer[3]) << 8) | buffer[2]) * COMPASS_RANGE_FACTOR;
  mz = (((int16_t)(buffer[5]) << 8) | buffer[4]) * COMPASS_RANGE_FACTOR;
}

float target_speed;
float target_rotation_speed;

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

void rampupDirectionControl(){
  float coefficient = 0.002;
  pid_speed_setpoint = pid_speed_setpoint * (1-coefficient) + target_speed * coefficient;
  rotation_speed_setpoint  = rotation_speed_setpoint * (1-coefficient) + target_rotation_speed * coefficient;

  if (abs(target_speed - pid_speed_setpoint) < (0.02*MAX_STEPS_PER_SECOND*MAX_BODY_SPEED_FACTOR)){
    pid_speed_setpoint = target_speed;
  }

  if (abs(target_rotation_speed - rotation_speed_setpoint) < (0.02*MAX_STEPS_PER_SECOND*MAX_BODY_SPEED_FACTOR)){
    rotation_speed_setpoint = target_rotation_speed;
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

void right(){
	target_speed = 0; target_rotation_speed = 0.5; 
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
      case '0': target_speed = 0; target_rotation_speed = 0; break;
      default: validInput = false;
     }

     if (validInput){
        target_speed *= MAX_STEPS_PER_SECOND * MAX_BODY_SPEED_FACTOR;
        target_rotation_speed *= MAX_STEPS_PER_SECOND * MAX_BODY_SPEED_FACTOR;
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
boolean last_motor_direction[2];
unsigned long last_motor_step_time; // the last time stepMotors() was called

float motor_step_iteration_interval; // how long between two calls of stepMotors()

long motor_steps[2];

const void stepMotors(int stepsMotor1, int stepsMotor2){  
  int directionMotor1 = stepsMotor1 >= 0 ? HIGH : LOW;
  int directionMotor2 = stepsMotor2 >= 0 ? HIGH : LOW;

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

  int counterMaximum = max(abs(stepsMotor1),abs(stepsMotor2));
  for (int i = 0; i < counterMaximum; i++) {
    if (i < abs(stepsMotor1)){
      digitalWriteFast(PIN_MOTOR_1_STEP, LOW);
    }
    if (i < abs(stepsMotor2)){
      digitalWriteFast(PIN_MOTOR_2_STEP, LOW);
    }
    
    _delay_us(MINIMUM_PIN_DELAY_MICROS);
    
    if (i < abs(stepsMotor1)){
      digitalWriteFast(PIN_MOTOR_1_STEP, HIGH);
    }
    if (i < abs(stepsMotor2)){
      digitalWriteFast(PIN_MOTOR_2_STEP, HIGH);
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
