/**
 * WristStepper.cpp
 * */

#include "Arduino.h"
#include "WristStepper.h"

#define STEPS_PER_REV 200
#define MICROS_PER_SECOND 1000000
#define DEFAULT_MAX_STEP_FREQ 4000
#define DEFUALT_P_GAIN 10

WristStepper::WristStepper(void (toggleStep)(void), const uint8_t dir_pin, const uint8_t* dmode_pins, const uint8_t enc_pin) :
            _dir_pin(dir_pin), _enc_pin(enc_pin), _dmode_pins(dmode_pins), _toggleStep(toggleStep){
    
    // Set defaults
    _max_step_freq = DEFAULT_MAX_STEP_FREQ;
    _p_gain = DEFUALT_P_GAIN;

    // Set initial setpoint
    this->sampleEncoder();
    _pos_setpoint = _encoder_pos;

    // Motors are stopped
    _stop_motor = true;

    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_dir_pin, LOW);
    pinMode(_enc_pin, INPUT);

    pinMode(_dmode_pins[0], OUTPUT);
    pinMode(_dmode_pins[1], OUTPUT);
    pinMode(_dmode_pins[2], OUTPUT);

    // Serial.print("Setting DMode...");
    this->setDMode(4);
    // Serial.println("...Done");
}

void WristStepper::sampleEncoder(){
    analogReadAveraging(20);
    _encoder_pos = analogRead(_enc_pin);
    analogReadAveraging(1);

}

bool WristStepper::setDMode(uint8_t microsteps){
    if(microsteps == 4){
        digitalWrite(_dmode_pins[0], LOW);
        digitalWrite(_dmode_pins[1], HIGH);
        digitalWrite(_dmode_pins[2], HIGH);
        return true;
    }else{
        return false;
    }

}

bool WristStepper::setSpeed(float speed){
    float step_freq;
    uint8_t dir;
    if(speed == 0){
        step_freq = 0;
        _stop_motor = true;
        _timer.end();
    }else{
        step_freq = abs(speed) * _max_step_freq;
        dir = (speed > 0) ? HIGH : LOW;
        digitalWrite(_dir_pin, dir);

        if(_stop_motor){
            _stop_motor = false;
            _timer.begin(_toggleStep, MICROS_PER_SECOND / (step_freq * 2.0));
        }else{
            _timer.update(MICROS_PER_SECOND / (step_freq * 2.0));
        }
    }
    return true;
}

float WristStepper::setPos(uint16_t pos_setpoint){
    
    if(pos_setpoint > 1023) return false;

    _pos_setpoint = pos_setpoint;
    this->sampleEncoder();
    int16_t error = pos_setpoint - _encoder_pos;

    // Take the shortest path
    if(abs(error) > (1024)/2){
        error = (1024 - abs(error)) * (error < 0 ? 1 : -1); // Go the opposite direction
    }

    float step_freq;

    if(abs(error) <= 1){
        step_freq = 0;
        _stop_motor = true;
        _timer.end();
    }else{
        step_freq = abs(error * _p_gain);
        digitalWrite(_dir_pin, (error > 0) ? LOW : HIGH); //TODO check that this would decrease error

        // Limit to max speed
        if(step_freq > _max_step_freq){
            step_freq = _max_step_freq;
        }

        if(_stop_motor){
            _stop_motor = false;
            _timer.begin(_toggleStep, MICROS_PER_SECOND / (step_freq * 2.0));
        }else{
            _timer.update(MICROS_PER_SECOND / (step_freq * 2.0));
        }
    }
    
    return step_freq;
}


bool WristStepper::setPosInc(int16_t incremental){

    // incrementals greater than 1023 should be OK
    int16_t setpoint = (this->getEncoderPos() + incremental) % 1024; // 1025 -> 1
    while(setpoint < 0){
         setpoint += 1024;  // -1 -> 1023
    }

    // Set position
    this->setPos(setpoint);

    return true;
}


