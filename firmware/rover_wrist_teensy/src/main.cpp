// Includes
#include <Arduino.h>
#include "WristStepper.h"


/* === Defines === */
#define STPR_1_STEP_PIN 5
#define STPR_2_STEP_PIN 6
#define STPR_1_DIR_PIN 7
#define STPR_2_DIR_PIN 8
#define ENCODER_1_PIN 14
#define ENCODER_2_PIN 15

#define MICROS_PER_SEC 1000000
#define STPR_TO_PITCH_GEAR_RATIO 1 //TODO
#define ENC_COUNTS_PER_DEG 1024 / 360

#define AIN_1_PIN 16
#define AIN_2_PIN 17

#define BLINK_PERIOD 500000

#define DEFAULT_P_GAIN 10

/* === Globals === */
const uint16_t ctrl_loop_freq = 100;
const uint32_t ctrl_loop_period = MICROS_PER_SEC / ctrl_loop_freq;  // in microseconds
const uint16_t max_step_freq = 4000;
uint64_t next_ctrl_loop = 0; // Micros for next control loop
uint64_t next_blink = 0;
uint8_t blink_state = LOW;
// bool stop_motor = true;
// IntervalTimer timer;

float wrist_pitch = 0;
float wrist_yaw = 0;


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

WristStepper stpr_1(toggleStep1, STPR_1_DIR_PIN, ENCODER_1_PIN, max_step_freq, DEFAULT_P_GAIN);
WristStepper stpr_2(toggleStep2, STPR_2_DIR_PIN, ENCODER_2_PIN, max_step_freq, DEFAULT_P_GAIN);

// void setWristPitch(float pitch){
//   float delta_deg = pitch - wrist_pitch;
//   int16_t enc_inc = delta_deg * STPR_TO_PITCH_GEAR_RATIO * ENC_COUNTS_PER_DEG;
//   stpr_1.setPosInc(enc_inc);
//   stpr_2.setPosInc(-1 * enc_inc); // TODO check signs
// }

// void setWristYaw(){

// }


void setup() {
  // init pins

  while(!Serial);
  
  pinMode(STPR_1_STEP_PIN, OUTPUT);
  pinMode(STPR_2_STEP_PIN, OUTPUT);

  //Temp joystick inputs
  pinMode(AIN_1_PIN, INPUT);
  pinMode(AIN_2_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

}

void loop() {

  uint64_t now = micros();

  if(now >= next_ctrl_loop){
    next_ctrl_loop = next_ctrl_loop + ctrl_loop_period;

    // Temp read joy stick
    uint16_t ain_1 = analogRead(AIN_1_PIN);
    uint16_t ain_2 = analogRead(AIN_2_PIN);

    // Sample encoders
    stpr_1.sampleEncoder();
    stpr_2.sampleEncoder();

    // Calculate error

    // Apply gain to calculate step freq

    // Map to (-1,1)
    float speed_1 = (ain_1 * (2.0/1023.0)) - 1;
    // float speed_2 = (ain_2 * (2.0/1023.0)) - 1;
    float speed_2 = speed_1;
    if(speed_1 < 0.1 && speed_1 > -0.1){
      speed_1 = 0;
    } 
    if(speed_2 < 0.1 && speed_2 > -0.1){
      speed_2 = 0;
    } 
    stpr_1.setSpeed(speed_1);
    stpr_2.setSpeed(speed_2);
    // if(ain_1 < 530 || ain_1 > 537){

    //   if(ain_1 < 530){
    //     step_freq = (530 - ain_1) * (max_step_freq / 530.0);
    //     digitalWriteFast(STPR_1_DIR_PIN, LOW);
    //   }else if(ain_1 > 537){
    //     step_freq = (ain_1 - 537) * (max_step_freq / (1024.0 - 537.0));
    //     digitalWriteFast(STPR_1_DIR_PIN, HIGH);
    //   }

    //   if(stop_motor){
    //     stop_motor = false;
    //     timer.begin(callback, MICROS_PER_SEC / (step_freq * 2.0));
    //   }else{
    //     timer.update(MICROS_PER_SEC / (step_freq * 2.0));
    //   }
    //   // analogWriteFrequency(STPR_1_STEP_PIN, step_freq);
    //   // analogWrite(STPR_1_STEP_PIN, 127);  // 50%

    // }else{
    //   step_freq = 0;
    //   stop_motor = true;
    //   timer.end();
    //   // analogWrite(STPR_1_STEP_PIN, 0);
    // }
    Serial.print("ain_1: "); Serial.print(ain_1);
    Serial.print("\tenc_1: "); Serial.print(analogRead(ENCODER_1_PIN));
    Serial.print("\tspeed: "); Serial.print(speed_1);
    Serial.print("\tperiod: "); Serial.print(MICROS_PER_SEC / abs(speed_1 * max_step_freq));
    Serial.println();

  }

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

}