#include "balancingRobotV3.h"
#include "melody.h"

// --------------------- START custom settings ---------------------
const float INITIAL_TARGET_ANGLE = 2.5; // approximation of the angle at which the bot stands still
const float STARTUP_ANGLE_TOLERANCE = 3.0; // the acceptable range of angle error for start balancing
const float TIPOVER_ANGLE_OFFSET = 35; // stop motors if bot has tipped over
const float MAX_ACCELLERATION = 100; // maximum acceleration in steps per second per loop iteration /* TODO optimize for faster acceleration or better stability [50:300]*/
const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT = 0.999; // how much to use gyro value compared to accerelometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START PID settings ---------------------
// pid for controlling angle
const float PID_ANGLE_P_GAIN = 5; // proportional pid gain /* TODO optimize [4:7] */
const float PID_ANGLE_I_GAIN = 0.05; // integral pid gain
const float PID_ANGLE_D_GAIN = -250; // differential pid gain
const float PID_ANGLE_I_MAX = 20; // limit for integral pid
const float PID_ANGLE_D_MAX = 20; // limit for differential pid

// pid for controlling speed
const float PID_SPEED_P_GAIN = 9; /* TODO optimize [5:15] */
const float PID_SPEED_I_GAIN = 0.01;
const float PID_SPEED_D_GAIN = 0;
const float PID_SPEED_I_MAX = 50000;
const float PID_SPEED_D_MAX = 10000;

// pid for controlling position
const float PID_POSITION_P_GAIN = -1;  /* TODO optimize [-1:-20] Hint: hat weniger Einfluss */
const float PID_POSITION_I_GAIN = 0;
const float PID_POSITION_D_GAIN = 0;
const float PID_POSITION_I_MAX = 20;
const float PID_POSITION_D_MAX = 20;
// --------------------- END PID settings ---------------------

unsigned long currentTime = millis();

// --------------------- START music variables  ---------------
boolean playMusicModeActive = false;
int lastMusicTone = -1;
unsigned long lastMusicTime = 0;
int *balancing_melody = MELODY_DUMMY; /* TODO your melody, see file melody.h */
int balancing_melody_size = sizeof(MELODY_DUMMY)/sizeof(int); /* TODO your melody */
// ---------------------- END music variables  ----------------

int count = 0;
void debugLoop(){
  if (count++ >= 50){
    count = 0;
    debugOutput();
  }
}

void debugOutput(){
    /* TODO Use this to debug if necessary! 
     * don't do any computational intesive stuff as this will affect the balancing
     */
    //Serial.println(bodySpeed);
  
}

void serialRead(){  
  //serialReadPosition();
  serialReadDirection();
}

boolean getAccelGyro = true;
void loop() {
  currentTime = millis();
  setPowerLimit();
  debugLoop();
  serialRead();

  if (playMusicModeActive){
    playMusic();
  }

  if (!enable) {
    waitForTargetAngle();
  }

  // every other iteration either read new accelgyro data or step motors
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

void serialReadDirection() {
  if (Serial.available() > 0) {
     lastDirectionInput = millis();
     mode = SPEED_MODE;
  
     char c = Serial.read();
     boolean validInput = true;

     switch (c){
      case 'F': forward(); break;
      case 'B': backward(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      case 'G': forward_left(); break;
      case 'I': forward_right(); break;
      case 'H': backward_left(); break;
      case 'J': backward_right(); break;
      case 'V': horn(); break;
      case 'v': /* Horn off */ break;
      case 'X': playMusicModeActive= true; break;
      case 'x': playMusicModeActive= false; break;
      case 'W': head_left(); break;
      case 'w': head_stop(); break;
      case 'U': head_right(); break;
      case 'u': head_stop(); break;
      case 'D': stop(); break;
      case 'S': stop(); break;
      case '0': stop(); break;
      /* TODO more cases for speed control */
      case '2': speed(); break; // set speed to 20% of maximum
      default: validInput = false;
     }

     if (validInput){
        target_speed *= MAX_STEPS_PER_SECOND * max_body_speed_factor;
        target_rotation_speed *= MAX_STEPS_PER_SECOND * max_body_speed_factor;
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

void forward(){
  target_speed = 1; target_rotation_speed = 0;
}

void backward(){
  target_speed = -1; target_rotation_speed = 0;
}

void left(){
  target_speed = 0; target_rotation_speed = -0.1; /* TODO optimize rotation speed */
}

void head_left(){
  /* TODO target_head_rotation_speed */
}

void right(){
  target_speed = 0; target_rotation_speed = 0.1; /* TODO optimize rotation speed */
}

void head_right(){
  /* TODO target_head_rotation_speed */
}

void forward_left(){
  /* TODO */
}

void forward_right(){
  /* TODO */
}

void backward_left(){
  /* TODO */
}

void backward_right(){
  /* TODO */
}

void stop(){
  target_speed = 0; target_rotation_speed = 0;
}

void head_stop(){
  target_speed = 0; target_head_rotation_speed = 0;
}

void speed(){
  /* TODO add support for multiple speed levels */
  max_body_speed_factor = 2.0 * MAX_BODY_SPEED_FACTOR_LIMIT/10.0;
}

void horn(){
  /* TODO use library balancingRobotV3.h to play 'warning tone' */
}

void playMusic(){
  /*  TODO implement
   *  This method will be called in the main loop about once per millisecond
   *  Don't use any delays, or bot cannot balance!!!
   */
  
  /* Variables to use:
   * lastMusicTone : last played tone index
   * lastMusicTime : time when lastMusicTone started
   * balancing_melody_size : size of balancing melody array, don't change!
   * balancing_melody : the array containing sets of tone + tone duration + pause duration
   * currentTime : the current time, set in loop, don't change!
   * 
   * play tone using:
   * tone(PIN_BUZZER, balancing_melody[lastMusicTone], balancing_melody[lastMusicTone+1]);
   * interface: tone(pin, tone frequency, tone duration)
   */
  
}

