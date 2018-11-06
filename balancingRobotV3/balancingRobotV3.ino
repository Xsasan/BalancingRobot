#include "balancingRobotV3.h"

// --------------------- START custom settings ---------------------
const float INITIAL_TARGET_ANGLE = 2.5;
const float STARTUP_ANGLE_TOLERANCE = 3.0;
const float TIPOVER_ANGLE_OFFSET = 35; // stop motors if bot has tipped over
const float MAX_ACCELLERATION = 150.0;
const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT = 0.999; // how much to use gyro value compared to accerelometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START PID settings ---------------------
const float PID_ANGLE_P_GAIN = 5;
const float PID_ANGLE_I_GAIN = 0.05;
const float PID_ANGLE_D_GAIN = -250;
const float PID_ANGLE_I_MAX = 20;
const float PID_ANGLE_D_MAX = 20;

const float PID_SPEED_P_GAIN = 11.0;
const float PID_SPEED_I_GAIN = 0.01;
const float PID_SPEED_D_GAIN = 0;
const float PID_SPEED_I_MAX = 50000;
const float PID_SPEED_D_MAX = 10000;

const float PID_POSITION_P_GAIN = -5;
const float PID_POSITION_I_GAIN = 0;
const float PID_POSITION_D_GAIN = 0;
const float PID_POSITION_I_MAX = 20;
const float PID_POSITION_D_MAX = 20;
// --------------------- END PID settings ---------------------

// ------------------------ START music ------------------------
// Note, On Duration, Off Duration
int melody[] = {
  NOTE_C4, 250, 30,
  NOTE_G3, 125, 30,
  NOTE_G3, 125, 30,
  NOTE_A3, 250, 30,
  NOTE_G3, 250, 300,
  NOTE_B3, 250, 30,
  NOTE_C4, 250, 30
};
// ------------------------  END music  ------------------------

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

  playMelody(melody);

  waitForTargetAngle();
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

