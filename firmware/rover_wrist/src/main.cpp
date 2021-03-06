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

#define STPR_2_STEP_PIN 0
#define STPR_2_DIR_PIN 1
#define STPR_2_DMODE_PINS {6, 5, 2}

#define CLAW_SERVO_PIN 7

#define ENCODER_1_PIN A9
#define ENCODER_2_PIN A8


/* === Constants === */
#define MICROS_PER_SEC 1000000
#define ENC_TO_PITCH_GEAR_RATIO 1
#define MAX_PITCH 55                // Degrees
#define MIN_PITCH -55
#define CLAW_MIN 123
#define CLAW_MAX 180
#define ENC_TO_YAW_GEAR_RATIO 1 //Is actually 2 but leave at one for simplicity.
#define ENC_COUNTS_PER_DEG 1024 / 360

// Values of encoders when pitch and yaw are at 0.
#define ENC_1_OFFSET 145
#define ENC_2_OFFSET 158

#define BLINK_PERIOD 500000
#define PRINT_PERIOD 100000 // 10 prints per second

#define DEFAULT_P_GAIN 25

/* === Globals === */
// Constants now that may be dynamic later
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

char printbuf[128];

// Revceived joint_state message callback
void setPositionCallback(const sensor_msgs::JointState& cmd_msg);
// Ros node handle
ros:: NodeHandle nh;
// Ros subscriber
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", setPositionCallback);

// Function called by timer at twice the step frequency to drive stepper 1
void toggleStep1() {
  static bool pulse_state = LOW;
  pulse_state = !pulse_state;
  digitalWriteFast(STPR_1_STEP_PIN, pulse_state);
}

// Function called by timer at twice the step frequency to drive stepper 2
void toggleStep2() {
  static bool pulse_state = LOW;
  pulse_state = !pulse_state;
  digitalWriteFast(STPR_2_STEP_PIN, pulse_state);
}

// use preset offsets to map encoder positions such that both are at 0 when pitch and yaw are at 0
int16_t getEncRelPos(uint16_t pos, uint16_t offset){
  // Map relative positions to (0,1023)
  int16_t rel_pos = pos - offset;
  if(rel_pos < 0) rel_pos += 1024;
  return rel_pos;
}

// Calculate the pitch based on current encoder positions
float getWristPitch(){
  int16_t enc_1_rel_pos = getEncRelPos(stpr_1->getEncoderPos(), ENC_1_OFFSET);
  int16_t enc_2_rel_pos = getEncRelPos(stpr_2->getEncoderPos(), ENC_2_OFFSET);
  
  // if encoder difference is greater than 512, one must have looped around. 
  // Add 1024 to the lower one.
  if(abs(enc_1_rel_pos - enc_2_rel_pos) > 512){
    if(enc_1_rel_pos > enc_2_rel_pos){
      enc_2_rel_pos += 1024;
    }else{
      enc_1_rel_pos += 1024;
    }
  }

  // As pitch increases, one encoder increases while the other decreases. Pitch is half the difference of the encoder values.
  float pitch = ((enc_1_rel_pos - enc_2_rel_pos) / 2.0) * (360.0/1024);
  return pitch;
}

// Calculate yaw based on current encoder positions
float getWristYaw(){
  int16_t enc_1_rel_pos = getEncRelPos(stpr_1->getEncoderPos(), ENC_1_OFFSET);
  int16_t enc_2_rel_pos = getEncRelPos(stpr_2->getEncoderPos(), ENC_2_OFFSET);

  // if encoder difference is greater than 512, one must have looped around. 
  // Add 1024 to the lower one.
  if(abs(enc_1_rel_pos - enc_2_rel_pos) > 512){
    if(enc_1_rel_pos > enc_2_rel_pos){
      enc_2_rel_pos += 1024;
    }else{
      enc_1_rel_pos += 1024;
    }
  }

  // As yaw increases, both encoders increase. Yaw is is the average of the encoder values.
  float yaw = ((enc_1_rel_pos + enc_2_rel_pos) / 2.0) * (360.0/1024);
  if(yaw > (360)){
    yaw = yaw - (360);
  }

  return yaw * ENC_TO_YAW_GEAR_RATIO;
}

// Update control loop setpoints. Call this at fixed frequency.
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

// Callback for JointState messages
void setPositionCallback(const sensor_msgs::JointState& cmd_msg){

  // TODO filter old/delayed

  float pitch = cmd_msg.position[3];
  float yaw = cmd_msg.position[4];
  float claw = cmd_msg.position[5];

  // Convert to degrees
  pitch = pitch * 360.0/(2*PI);
  yaw = yaw * 360.0/(2*PI);
  // claw = claw * 360.0/(2*PI);

  // Map yaw to [0, 360)
  while(yaw < 0) yaw += 360;
  while(yaw >= 360) yaw -= 360;

  // Map pitch to [-180, 180)
  while(pitch < -180) pitch += 360;
  while(pitch >= 180) pitch -= 360;

  // Map claw to [0, 2pi]
  while(claw < 0) claw += 2*PI;
  while(claw >= 2*PI) claw -= 2*PI;

  // Map claw from [0, 2pi) to [MIN, MAX)
  claw = (claw * (CLAW_MAX - CLAW_MIN) / (2 * PI)) + CLAW_MIN;

  // Claw endpoint safety
  if(claw < CLAW_MIN) claw = CLAW_MIN;
  if(claw > CLAW_MAX) claw = CLAW_MAX;

  yaw_setpoint = yaw;
  pitch_setpoint = pitch;
  claw_setpoint = claw;

}

void setup() {

  // Wait for serial node to connect
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
  claw_setpoint = 150;
}

void loop() {

  uint64_t now = micros();

  // Control loop
  if(now >= next_ctrl_loop){
    next_ctrl_loop = next_ctrl_loop + ctrl_loop_period;

    setWrist(yaw_setpoint, pitch_setpoint);
    claw_servo.write(claw_setpoint);
  }

  // print loop
  if(now >= next_print && Serial){
    next_print += PRINT_PERIOD;

    snprintf(printbuf, 128, "pitch: %.2f\tsp: %.2f\tyaw: %.2f\tsp: %.2f\tclaw: %.2f\te1 rel pos: %i\te2 rel pos: %i", getWristPitch(), pitch_setpoint, getWristYaw(), yaw_setpoint, claw_setpoint, getEncRelPos(stpr_1->getEncoderPos(), ENC_1_OFFSET), getEncRelPos(stpr_2->getEncoderPos(), ENC_2_OFFSET));
    nh.loginfo(printbuf);
  }

  // LED loop
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