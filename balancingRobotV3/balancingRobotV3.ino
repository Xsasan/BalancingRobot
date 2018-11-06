#include "balancingRobotV3.h"

// --------------------- START custom settings ---------------------
const float INITIAL_TARGET_ANGLE = 2.5; // approximation of the angle at which the bot stands still
const float STARTUP_ANGLE_TOLERANCE = 3.0; // the acceptable range of angle error for start balancing
const float TIPOVER_ANGLE_OFFSET = 35; // stop motors if bot has tipped over
const float MAX_ACCELLERATION = 150.0; // maximum acceleration in steps per second per loop iteration
const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT = 0.999; // how much to use gyro value compared to accerelometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START PID settings ---------------------
// pid for controlling angle
const float PID_ANGLE_P_GAIN = 5; // proportional pid gain
const float PID_ANGLE_I_GAIN = 0.05; // integral pid gain
const float PID_ANGLE_D_GAIN = -250; // differential pid gain
const float PID_ANGLE_I_MAX = 20; // limit for integral pid
const float PID_ANGLE_D_MAX = 20; // limit for differential pid

// pid for controlling speed
const float PID_SPEED_P_GAIN = 11.0;
const float PID_SPEED_I_GAIN = 0.01;
const float PID_SPEED_D_GAIN = 0;
const float PID_SPEED_I_MAX = 50000;
const float PID_SPEED_D_MAX = 10000;

// pid for controlling position
const float PID_POSITION_P_GAIN = -5;
const float PID_POSITION_I_GAIN = 0;
const float PID_POSITION_D_GAIN = 0;
const float PID_POSITION_I_MAX = 20;
const float PID_POSITION_D_MAX = 20;
// --------------------- END PID settings ---------------------

// ------------------------ START music ------------------------
// Note, On duration, Off duration
int startup_melody[] = {
  NOTE_C4, 250, 30,
  NOTE_G3, 125, 30,
  NOTE_G3, 125, 30,
  NOTE_A3, 250, 30,
  NOTE_G3, 250, 300,
  NOTE_B3, 250, 30,
  NOTE_C4, 250, 30
};
int startup_melody_size = sizeof(startup_melody);

int balancing_melody[] = {
  NOTE_A4, 500, 30,
  NOTE_A4, 500, 30,
  NOTE_A4, 500, 30,
  NOTE_F4, 350, 30,
  NOTE_C5, 150, 30,
  NOTE_A4, 500, 30,
  NOTE_F4, 350, 30,
  NOTE_C5, 150, 30,
  NOTE_A4, 650, 500,
  
  NOTE_E5, 500, 30,
  NOTE_E5, 500, 30,
  NOTE_E5, 500, 30,
  NOTE_F5, 350, 30,
  NOTE_C5, 150, 30,
  NOTE_GS4, 500, 30,
  NOTE_F4, 350, 30,
  NOTE_C5, 150, 30,
  NOTE_A4, 650, 500,
  
  NOTE_A5, 500, 30,
  NOTE_A4, 300, 30,
  NOTE_A4, 150, 30,
  NOTE_A5, 500, 30,
  NOTE_GS5, 325, 30,
  NOTE_G5, 175, 30,
  NOTE_FS5, 125, 30,
  NOTE_F5, 125, 30,
  NOTE_FS5, 250, 325,
  
  455, 250, 30,
  NOTE_DS5, 500, 30,
  NOTE_D5, 325, 30,
  NOTE_CS5, 175, 30,
  NOTE_C5, 125, 30,
  NOTE_AS4, 125, 30,
  NOTE_C5, 250, 350,
  
  NOTE_F4, 250, 30,
  NOTE_GS4, 500, 30,
  NOTE_F4, 350, 30,
  NOTE_A4, 125, 30,
  NOTE_C5, 500, 30,
  NOTE_A4, 375, 30,
  NOTE_C5, 125, 30,
  NOTE_E5, 650, 500,
  
  NOTE_A5, 500, 30,
  NOTE_A4, 300, 30,
  NOTE_A4, 150, 30,
  NOTE_A5, 500, 30,
  NOTE_GS5, 325, 30,
  NOTE_G5, 175, 30,
  NOTE_FS5, 125, 30,
  NOTE_F5, 125, 30,
  NOTE_FS5, 250, 325,
  
  455, 250, 30,
  NOTE_DS5, 500, 30,
  NOTE_D5, 325, 30,
  NOTE_CS5, 175, 30,
  NOTE_C5, 125, 30,
  NOTE_AS4, 125, 30,
  NOTE_C5, 250, 350,
  
  NOTE_F4, 250, 30,
  NOTE_GS4, 500, 30,
  NOTE_F4, 375, 30,
  NOTE_C5, 125, 30,
  NOTE_A4, 500, 30,
  NOTE_F4, 375, 30,
  NOTE_C5, 125, 30,
  NOTE_A4, 650, 650
};
int balancing_melody_size = sizeof(balancing_melody);
// ------------------------  END music  ------------------------

unsigned long currentTime = millis();

int count = 0;
void debugLoop(){
  if (count++ >= 50){
    count = 0;
    debugOutput();
  }
}

void debugOutput(){
    /* Use this to debug! 
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

  playMusic();

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

int lastMusicTone = -1;
unsigned long lastMusicTime = 0;
void playMusic(){
  if (lastMusicTone >= sizeof(balancing_melody)/sizeof(int)){
    lastMusicTone = -1;
  }
  
  if (lastMusicTone < 0){
    lastMusicTone = 0;
    lastMusicTime = currentTime;
    tone(PIN_BUZZER, balancing_melody[lastMusicTone], balancing_melody[lastMusicTone+1]);
    return;
  }

  if (currentTime-lastMusicTime < balancing_melody[lastMusicTone+1]){
    return;   
  }

  if (currentTime-lastMusicTime >= balancing_melody[lastMusicTone+1]){
    if (currentTime-lastMusicTime > balancing_melody[lastMusicTone+1] + balancing_melody[lastMusicTone+2]){
      lastMusicTone += 3;
      if (lastMusicTone < sizeof(balancing_melody)/sizeof(int)){
        lastMusicTime = currentTime;
        tone(PIN_BUZZER, balancing_melody[lastMusicTone], balancing_melody[lastMusicTone+1]);
      }
    }
    return;
  }
}

