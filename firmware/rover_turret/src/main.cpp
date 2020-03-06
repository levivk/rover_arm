#include <Arduino.h>
#include "ros.h"
#include "sensor_msgs/JointState.h"


#define STPR_STEP_PIN 14
#define STPR_DIR_PIN 15
#define LEFT_LIM_PIN 3
#define RIGHT_LIM_PIN 4

#define MAX_STEP_FREQ 1000
#define MICROS_PER_SECOND 1000000
#define BLINK_PERIOD 500000
#define PRINT_PERIOD 100000

/* === Globals === */

void setPositionCallback(const sensor_msgs::JointState& cmd_msg);
ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", setPositionCallback);
IntervalTimer timer;
float step_freq = 0;
uint8_t dir = 0;
bool motor_stopped = true;

// Function to be called by the timer at twice the step frequency 
void toggleStep(){
  static bool pulse_state = LOW;
  pulse_state = !pulse_state;
  digitalWriteFast(STPR_STEP_PIN, pulse_state);
}

// Called on receive of joint_state message
void setPositionCallback(const sensor_msgs::JointState& cmd_msg){

  // Because turret control is open loop (for now) send velocity through joint_state position
  float turret_vel = cmd_msg.position[0];   // value is [-2,2]

  // TODO test with limit switches
  bool left_limit_hit = digitalRead(LEFT_LIM_PIN);
  bool right_limit_hit = digitalRead(RIGHT_LIM_PIN);

  bool dont_go_left = (turret_vel < 0) && left_limit_hit;
  bool dont_go_right = (turret_vel > 0) && right_limit_hit;
  
  if((turret_vel == 0) || dont_go_left || dont_go_right){
    step_freq = 0;
    motor_stopped = true;
    timer.end();
  }else{
    // Map command velocity [-2,2] to step frequency (0,MAX)
    step_freq = abs(turret_vel) * MAX_STEP_FREQ / 2.0;
    dir = (turret_vel > 0) ? HIGH : LOW;
    digitalWrite(STPR_DIR_PIN, dir);

    // Restart or update step timer
    if(motor_stopped){
      motor_stopped = false;
      timer.begin(toggleStep, MICROS_PER_SECOND / (step_freq * 2.0));
    }else{
      timer.update(MICROS_PER_SECOND / (step_freq * 2.0));
    }
  }
}

void setup() {

  // Wait for ROS node to connect
  while(!Serial);

  nh.initNode();
  nh.subscribe(sub);

  pinMode(STPR_STEP_PIN, OUTPUT);
  pinMode(STPR_DIR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //TODO set pinmode for limit switches
}

void loop() {
  uint64_t now = micros();
  static uint64_t next_blink = 0;
  static uint64_t next_print = 0;
  static uint8_t blink_state = LOW;

  // Blink the LED
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

  // Log info at fixed interval
  if(now >= next_print){
    next_print += PRINT_PERIOD;
    static char buf[128];
    snprintf(buf, 128, "freq: %f\tdir: %i", step_freq, dir);
    nh.loginfo(buf);
  }

  // Do the ROS stuff
  nh.spinOnce();
}