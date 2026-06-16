#pragma once
#include <cstdint>
#include "OscillatorState.h"

// This class holds all state and methods related to oscillators

class Adafruit_PWMServoDriver;

class Oscillator
{
public:
    Oscillator(unsigned int servoMin = 100, unsigned int servoMax = 450, const OscillatorParams &parameters = {});

    void setPWM(Adafruit_PWMServoDriver &pwm);

    // Updates the current state of the motor based on the elapsed time (in seconds)
    void update(double dt);

    void reset();

    // Adjusts a parameter of this oscillator using information obtained from an OscillatorUpdateCommand
    // Assuming the struct is used as a Serial communication format, one can easily reinterpret a 12byte char[] as an OscillatorUpdateCommand
    void setParams(const OscillatorParams &command);

    const OscillatorState &getState() const;

    const uint8_t id;

private:
    Adafruit_PWMServoDriver *pwm = nullptr;

    const float AMPLITUDE_CHANGE_FACTOR = 1;
    const float OFFSET_CHANGE_FACTOR = 0.5;
    const float FREQ_CHANGE_FACTOR = 0.5;
    const float PHASESHIFT_CHANGE_FACTOR = 1;

    OscillatorParams params;
    OscillatorState state;

    const OscillatorParams defaultParams;

    unsigned int servoMin, servoMax;
};
