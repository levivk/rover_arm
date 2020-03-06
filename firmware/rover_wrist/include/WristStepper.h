/**
 * WristStepper.h
 * */

#ifndef WristStepper_h
#define WristStepper_h

#include <Arduino.h>


class WristStepper{
    public:
        WristStepper(void (toggleStep)(void), const uint8_t dir_pin, const uint8_t* dmode_pins, const uint8_t enc_pin);
        bool setSpeed(float speed);
        float setPos(uint16_t pos_setpoint); // position according to encoder (0 to 1023)
        bool setPosInc(int16_t incremental);   // Incremental position movement
        void sampleEncoder();   // Normally do not need to use as position control will manage


        uint16_t getEncoderPos() {return _encoder_pos;}
        void setPgain(float p) {_p_gain = p;}   // Proportional gain
        bool setDMode(uint8_t micro_steps);     // Set microstep resolution. Parameter is (micro)steps per step
        void setMaxStepFreq(const uint16_t max_step_freq){_max_step_freq = max_step_freq;}  // Speed limit
    
    private:
        IntervalTimer _timer;
        const uint8_t _dir_pin, _enc_pin;
        const uint8_t* _dmode_pins;
        uint16_t _max_step_freq;
        bool _stop_motor = false;
        uint16_t _encoder_pos, _pos_setpoint;
        float _p_gain;


        // Callback used to appease requirements of timer.begin
        // Also allows for benifit of digitalWriteFast()
        void (*_toggleStep) ();

};

#endif