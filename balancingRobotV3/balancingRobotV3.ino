#include "balancingRobotV3.h"
#include "melody.h"

// --------------------- START custom settings ---------------------
const float INITIAL_TARGET_ANGLE = 2.5; // approximation of the angle at which the bot stands still
const float STARTUP_ANGLE_TOLERANCE = 3.0; // the acceptable range of angle error for start balancing
const float TIPOVER_ANGLE_OFFSET = 35; // stop motors if bot has tipped over
const float MAX_ACCELLERATION = 200.0; // maximum acceleration in steps per second per loop iteration
const float COMPLEMENTARY_FILTER_GYRO_COEFFICIENT = 0.999; // how much to use gyro value compared to accerelometer value
// ---------------------  END custom settings  ---------------------

// --------------------- START PID settings ---------------------
// pid for controlling angle
const float PID_ANGLE_P_GAIN = 6; // proportional pid gain
const float PID_ANGLE_I_GAIN = 0.05; // integral pid gain
const float PID_ANGLE_D_GAIN = -250; // differential pid gain
const float PID_ANGLE_I_MAX = 20; // limit for integral pid
const float PID_ANGLE_D_MAX = 20; // limit for differential pid

// pid for controlling speed
const float PID_SPEED_P_GAIN = 13.0;
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

unsigned long currentTime = millis();

// --------------------- START music variables  ---------------
boolean playMusicModeActive = false;
int lastMusicTone = -1;
unsigned long lastMusicTime = 0;
int *balancing_melody = MELODY_PIRATE_CARIBBEAN;
int balancing_melody_size = sizeof(MELODY_PIRATE_CARIBBEAN)/sizeof(int);
// ---------------------- END music variables  ----------------

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
  target_speed = 0; target_rotation_speed = -0.8;
}

void head_left(){
  target_speed = 0; target_head_rotation_speed = -0.8;
}

void right(){
  target_speed = 0; target_rotation_speed = 0.8; 
}

void head_right(){
  target_speed = 0; target_head_rotation_speed = 0.8;
}

void forward_left(){
  target_speed = 1.0; target_rotation_speed = -0.4;
}

void forward_right(){
  target_speed = 1.0; target_rotation_speed = 0.4;
}

void backward_left(){
  target_speed = -1.0; target_rotation_speed = 0.4;
}

void backward_right(){
  target_speed = -1.0; target_rotation_speed = -0.4;
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

void horn(){
  playWarningToneWithDuration(250,750);
}

void playMusic(){
  if (lastMusicTone >= balancing_melody_size){
    lastMusicTone = -1;
  }
  
  if (lastMusicTone < 0){
    lastMusicTone = 0;
    lastMusicTime = currentTime;
    if (balancing_melody[lastMusicTone] == 0){
      return;
    }
    tone(PIN_BUZZER, balancing_melody[lastMusicTone], balancing_melody[lastMusicTone+1]);
    return;
  }

  if (currentTime-lastMusicTime < balancing_melody[lastMusicTone+1]){
    return;   
  }

  if (currentTime-lastMusicTime >= balancing_melody[lastMusicTone+1]){
    if (currentTime-lastMusicTime > balancing_melody[lastMusicTone+1] + balancing_melody[lastMusicTone+2]){
      lastMusicTone += 3;
      if (lastMusicTone < balancing_melody_size){
        lastMusicTime = currentTime;
        if (balancing_melody[lastMusicTone] == 0){
          return;
        }
        tone(PIN_BUZZER, balancing_melody[lastMusicTone], balancing_melody[lastMusicTone+1]);
      }
    }
    return;
  }
}

