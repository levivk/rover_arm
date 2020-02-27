// Includes
#include <Arduino.h>
#include "ros.h"
#include "sensor_msgs/JointState.h"
#include "WristStepper.h"
#include <Servo.h>


/* === Pin assignments === */
#define STPR_1_STEP_PIN 18
#define STPR_1_DIR_PIN 17
#define STPR_1_DMODE_PINS {14, 15, 16}
// #define STPR_1_DMODE1_PIN 15
// #define STPR_1_DMODE2_PIN 16

#define STPR_2_STEP_PIN 0
#define STPR_2_DIR_PIN 1
#define STPR_2_DMODE_PINS {6, 5, 2}
// #define STPR_2_DMODE1_PIN 5
// #define STPR_2_DMODE2_PIN 2

#define CLAW_SERVO_PIN 7

#define ENCODER_1_PIN A9
#define ENCODER_2_PIN A8

/* === Constants === */

#define MICROS_PER_SEC 1000000
#define ENC_TO_PITCH_GEAR_RATIO 1
#define MAX_PITCH 35                // Degrees
#define MIN_PITCH -45
#define CLAW_MIN 0
#define CLAW_MAX 90
#define ENC_TO_YAW_GEAR_RATIO 1 //TODO
#define ENC_COUNTS_PER_DEG 1024 / 360

#define ENC_1_OFFSET 173
#define ENC_2_OFFSET 321

// #define AIN_1_PIN A6
// #define AIN_2_PIN 17

#define BLINK_PERIOD 500000
#define PRINT_PERIOD 100000 // 10 prints per second

#define DEFAULT_P_GAIN 25

/* === Globals === */
const uint16_t ctrl_loop_freq = 100;
const uint32_t ctrl_loop_period = MICROS_PER_SEC / ctrl_loop_freq;  // in microseconds
const uint16_t max_step_freq = 4000;
const uint8_t stpr_1_dmode_pins[] = STPR_1_DMODE_PINS;
const uint8_t stpr_2_dmode_pins[] = STPR_2_DMODE_PINS;
uint64_t next_ctrl_loop = 0; // Micros for next control loop
uint64_t next_blink = 0;
uint64_t next_print = 0;
uint8_t blink_state = LOW;

float pitch_setpoint = 0;
float yaw_setpoint = 0;
float claw_setpoint = 0;

WristStepper* stpr_1;
WristStepper* stpr_2;
Servo claw_servo;

// Revceived message callback
void setPositionCallback(const sensor_msgs::JointState& cmd_msg);
// Ros node handle
ros:: NodeHandle nh;
// Ros subscriber
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", setPositionCallback);



void toggleStep1() {
  static bool pulse_state = LOW;
  pulse_state = !pulse_state;
  digitalWriteFast(STPR_1_STEP_PIN, pulse_state);
}

void toggleStep2() {
  static bool pulse_state = LOW;
  pulse_state = !pulse_state;
  digitalWriteFast(STPR_2_STEP_PIN, pulse_state);
}

int16_t getEncRelPos(uint16_t pos, uint16_t offset){
  // Map relative positions to (-511,512)
  int16_t rel_pos = pos - offset;
  if(rel_pos < 0) rel_pos += 1024;
  // if(rel_pos > (1024/2)) rel_pos -= 1024;
  return rel_pos;
}

float getWristPitch(){
  int16_t enc_1_rel_pos = getEncRelPos(stpr_1->getEncoderPos(), ENC_1_OFFSET);
  int16_t enc_2_rel_pos = getEncRelPos(stpr_2->getEncoderPos(), ENC_2_OFFSET);
  
  // // subtract positions for same sign, add for opposite sign
  // float pitch;
  // if( ((enc_1_rel_pos < 0)?-1:1) == ((enc_2_rel_pos < 0)?-1:1)){
  //   pitch = ((enc_1_rel_pos - enc_2_rel_pos) / 2.0) * (360.0/1023);
  // }else{
  //   pitch = ((enc_1_rel_pos + enc_2_rel_pos) / 2.0) * (360.0/1023);
  // }

  // if encoder difference is greater than 512, one must have looped around. 
  // Add 1023 to the lower one.
  if(abs(enc_1_rel_pos - enc_2_rel_pos) > 512){
    if(enc_1_rel_pos > enc_2_rel_pos){
      enc_2_rel_pos += 1024;
    }else{
      enc_1_rel_pos += 1024;
    }
  }
  float pitch = ((enc_1_rel_pos - enc_2_rel_pos) / 2.0) * (360.0/1024);
  return pitch;
}

float getWristYaw(){
  int16_t enc_1_rel_pos = getEncRelPos(stpr_1->getEncoderPos(), ENC_1_OFFSET);
  int16_t enc_2_rel_pos = getEncRelPos(stpr_2->getEncoderPos(), ENC_2_OFFSET);

  // // add positions for same sign, subtract for opposite sign
  // float yaw;
  // if( ((enc_1_rel_pos < 0)?-1:1) == ((enc_2_rel_pos < 0)?-1:1)){
  //   yaw = ((enc_1_rel_pos + enc_2_rel_pos) / 2.0) * (360.0/1023) * ENC_TO_YAW_GEAR_RATIO;
  // }else{
  //   yaw = ((enc_1_rel_pos - enc_2_rel_pos) / 2.0) * (360.0/1023) * ENC_TO_YAW_GEAR_RATIO;
  // }

  // if encoder difference is greater than 512, one must have looped around. 
  // subtract 1023 from the higher one.
  if(abs(enc_1_rel_pos - enc_2_rel_pos) > 512){
    if(enc_1_rel_pos > enc_2_rel_pos){
      enc_2_rel_pos += 1024;
    }else{
      enc_1_rel_pos += 1024;
    }
  }
  float yaw = ((enc_1_rel_pos + enc_2_rel_pos) / 2.0) * (360.0/1024) * ENC_TO_YAW_GEAR_RATIO;
  if(yaw > 360){
    yaw = yaw - 360;
  }
  return yaw;
}


bool setWrist(float yaw_setpoint, float pitch_setpoint){
  // Pitch endpoints
  if( pitch_setpoint > MAX_PITCH){
    pitch_setpoint = MAX_PITCH;
  }else if(pitch_setpoint < MIN_PITCH){
    pitch_setpoint = MIN_PITCH;
  }

  float yaw_delta_deg = yaw_setpoint - getWristYaw();
  float pitch_delta_deg = pitch_setpoint - getWristPitch();

  // Take shortest yaw path
  if(abs(yaw_delta_deg) > 360/2){
    yaw_delta_deg = (360 - abs(yaw_delta_deg)) * (yaw_delta_deg < 0 ? 1 : -1);  // opposite direction
  }

  int16_t yaw_enc_inc = yaw_delta_deg * ENC_TO_YAW_GEAR_RATIO * ENC_COUNTS_PER_DEG / 2;
  int16_t pitch_enc_inc = pitch_delta_deg * ENC_TO_PITCH_GEAR_RATIO * ENC_COUNTS_PER_DEG / 2;

  int16_t enc_1_inc = yaw_enc_inc + pitch_enc_inc;
  int16_t enc_2_inc = yaw_enc_inc - pitch_enc_inc;

  stpr_1->setPosInc(enc_1_inc);
  stpr_2->setPosInc(enc_2_inc);

  return true;
}


// float setWristPitch(float pitch_setpoint, bool exectute){
//   // Endpoints
//   if( pitch_setpoint > MAX_PITCH){
//     pitch_setpoint = MAX_PITCH;
//   }else if(pitch_setpoint < MIN_PITCH){
//     pitch_setpoint = MIN_PITCH;
//   }

//   float delta_deg = pitch_setpoint - getWristPitch();

//   if(exectute){
//     int16_t enc_inc = delta_deg * ENC_TO_PITCH_GEAR_RATIO * ENC_COUNTS_PER_DEG / 2;
//     // Steppers go opposite ways for pitch
//     stpr_1->setPosInc(enc_inc);
//     stpr_2->setPosInc(-1 * enc_inc);
//   }
//   last_pitch = pitch_setpoint;
//   return delta_deg;
// }

// float setWristYaw(float yaw_setpoint, bool execute){
//   float delta_deg = yaw_setpoint - getWristYaw();

//   // Take shortest path
//   if(abs(delta_deg) > 360/2){
//     delta_deg = (360 - abs(delta_deg)) * (delta_deg < 0 ? 1 : -1);  // opposite direction
//   }

//   if(execute){
//     int16_t enc_inc = delta_deg * ENC_TO_YAW_GEAR_RATIO * ENC_COUNTS_PER_DEG / 2;
//     // Steppers go same way for yaw
//     stpr_1->setPosInc(enc_inc);
//     stpr_2->setPosInc(enc_inc);
//   }
//   last_yaw = yaw_setpoint;
//   return delta_deg;
// }

void setPositionCallback(const sensor_msgs::JointState& cmd_msg){

  // TODO filter old

  float pitch = cmd_msg.position[3];
  float yaw = cmd_msg.position[4];
  float claw = cmd_msg.position[5];

  // Convert to degrees
  pitch = pitch * 360.0/(2*PI);
  yaw = yaw * 360.0/(2*PI);
  claw = claw * 360.0/(2*PI);

  // Map yaw to [0, 360)
  while(yaw < 0) yaw += 360;
  while(yaw >= 360) yaw -= 360;

  // Map pitch to [-180, 180)
  while(pitch < -180) pitch += 360;
  while(pitch >= 180) pitch -= 360;

  // Map claw to [0, 180)
  while(claw < 0) claw += 180;
  while(claw >= 180) claw -= 180;

  // Claw endpoints
  if(claw < CLAW_MIN) claw = CLAW_MIN;
  if(claw > CLAW_MAX) claw = CLAW_MAX;

}

void setup() {

  while(!Serial);

  // Init ROS
  nh.initNode();
  nh.subscribe(sub);

  nh.loginfo("Starting...");

  // init pins
  pinMode(STPR_1_STEP_PIN, OUTPUT);
  pinMode(STPR_2_STEP_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // init steppers
  stpr_1 = new WristStepper(toggleStep1, STPR_1_DIR_PIN, stpr_1_dmode_pins, ENCODER_1_PIN);
  stpr_2 = new WristStepper(toggleStep2, STPR_2_DIR_PIN, stpr_2_dmode_pins, ENCODER_2_PIN);
  stpr_1->setMaxStepFreq(max_step_freq);
  stpr_2->setMaxStepFreq(max_step_freq);
  stpr_1->setPgain(DEFAULT_P_GAIN);
  stpr_2->setPgain(DEFAULT_P_GAIN);

  // init claw servo
  claw_servo.attach(CLAW_SERVO_PIN);

  // Initial joint states
  yaw_setpoint = getWristYaw();
  pitch_setpoint = getWristPitch();
  claw_setpoint = 50;
}

void loop() {

  uint64_t now = micros();

  if(now >= next_ctrl_loop){
    next_ctrl_loop = next_ctrl_loop + ctrl_loop_period;

    setWrist(yaw_setpoint, pitch_setpoint);
    claw_servo.write(claw_setpoint);


  }

  // print loop
/*
  if(now >= next_print && Serial){
    next_print += PRINT_PERIOD;
    // Serial.println(ain_1);
    // Serial.print(stpr_2.getEncoderPos());

    // Serial.print("\tencoder 1 pos: "); Serial.print(stpr_1->getEncoderPos());
    // Serial.print("\tencoder 2 pos: "); Serial.print(stpr_2->getEncoderPos());
    // Serial.print("\tpitch: "); Serial.print(getWristPitch());
    // Serial.print("\tyaw: "); Serial.print(getWristYaw());

    // Serial.println();
  }
*/
  if(now >= next_blink){
    next_blink = next_blink + BLINK_PERIOD;
    if(blink_state == HIGH){
      blink_state = LOW;
      digitalWriteFast(LED_BUILTIN, blink_state);
    }else{
      blink_state = HIGH;
      digitalWriteFast(LED_BUILTIN, blink_state);
    }
  }

  // Do the ROS stuff
  nh.spinOnce();

}