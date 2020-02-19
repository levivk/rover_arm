/**
 * WristStepper.h
 * */

#ifndef WristStepper_h
#define WristStepper_h

#include <Arduino.h>

// #define

class WristStepper{
    public:
        WristStepper(void (toggleStep)(void), uint8_t dir_pin, uint8_t enc_pin, uint16_t max_step_freq, float p_gain);
        bool setSpeed(float speed);
        bool setPos(uint16_t pos_setpoint); // position according to encoder (0 to 1023)

        void sampleEncoder() {_encoder_pos = analogRead(_enc_pin);}
        uint16_t getEncoderPos() {return _encoder_pos;}
        void setPgain(float p) {_p_gain = p;}

    
    private:
        IntervalTimer _timer;
        uint8_t _dir_pin, _enc_pin;
        uint16_t _max_step_freq;
        bool _stop_motor = false;
        uint16_t _encoder_pos, _pos_setpoint;
        float _p_gain;


        // Callback used to appease requirements of timer.begin
        // Also allows for benifit of digitalWriteFast()
        void (*_toggleStep) ();

};

#endif