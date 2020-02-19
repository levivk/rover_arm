#include <Arduino.h>

#define SAMPLE_FREQ 100
#define MICROS_IN_SEC 1000000
#define MICROS_PER_SAMPLE 10000
#define MAX_STEP_FREQ 4000         // Speed limit

#define TIMER_2_PRESCALE 256

uint32_t now = 0;
uint32_t nextSample = 0;
uint32_t lastPulseEdge = 0;

uint8_t pulseState = LOW;
uint32_t stepPeriod = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  now = micros();

  // Sample at [100] hz
  if(now >= nextSample){
    nextSample = nextSample + MICROS_PER_SAMPLE;
    uint16_t ain = analogRead(0);

    double stepFreq;
    if(ain < 530){
      stepFreq = (530 - ain) * 7.54;   // 2000 / 530
    }else if(ain > 540){
      stepFreq = (ain - 540) * 8.28;  // 2000 / 483
    }else{
      stepFreq = 1;
    }

    stepPeriod = MICROS_IN_SEC / stepFreq;
    Serial.print("freq: "); Serial.print(stepFreq);
    Serial.print("\t\tperiod: "); Serial.print(stepPeriod);
    Serial.print("\t\tstate: "); Serial.println(pulseState);
  }

  if(now >= (lastPulseEdge + (stepPeriod/2))){
    lastPulseEdge = lastPulseEdge + (stepPeriod/2);
    if(stepPeriod == MICROS_IN_SEC){
      pulseState = LOW;
    }else if(pulseState == LOW){
      pulseState = HIGH;
    }else if(pulseState == HIGH){
      pulseState = LOW;
    }

    digitalWrite(2, pulseState);
  }
  
  

}