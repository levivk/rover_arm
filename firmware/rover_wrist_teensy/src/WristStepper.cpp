/**
 * WristStepper.cpp
 * */

#include "Arduino.h"
#include "WristStepper.h"

#define STEPS_PER_REV 200
#define MICROS_PER_SECOND 1000000

WristStepper::WristStepper(void (toggleStep)(void), uint8_t dir_pin, uint8_t enc_pin, uint16_t max_step_freq, float p_gain){
    _toggleStep = toggleStep;
    _dir_pin = dir_pin;
    _enc_pin = enc_pin;
    // _max_step_freq = max_rpm * STEPS_PER_REV / 60.0;
    _max_step_freq = max_step_freq;
    _p_gain = p_gain;

    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_dir_pin, LOW);
    pinMode(_enc_pin, INPUT);

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

bool WristStepper::setPos(uint16_t pos_setpoint){
    
    if(pos_setpoint > 1023) return false;

    _pos_setpoint = pos_setpoint;
    int16_t error = pos_setpoint - _encoder_pos;
    float step_freq;

    if(error == 0){
        step_freq = 0;
        _stop_motor = true;
        _timer.end();
    }else{
        step_freq = error * _p_gain;
        digitalWrite(_dir_pin, (step_freq > 0) ? HIGH : LOW); //TODO check that this would decrease error

        if(_stop_motor){
            _stop_motor = false;
            _timer.begin(_toggleStep, MICROS_PER_SECOND / (step_freq * 2.0));
        }else{
            _timer.update(MICROS_PER_SECOND / (step_freq * 2.0));
        }
    }

    float step_freq = error * _p_gain;

    
    return true;
}

// void WristStepper::toggleStep(){
//     static uint8_t pulse_state = LOW;
//     pulse_state = !pulse_state;
//     digitalWrite(_step_pin, pulse_state);
// }

