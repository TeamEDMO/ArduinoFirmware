#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "OscillatorState.h"

// This class holds all state and methods related to oscillators
class Oscillator
{
public:
    Oscillator(unsigned int servoMin = 100, unsigned int servoMax = 450, float amplitude = 0, float offset = 90, float phaseShift = 0, float frequency = 0)
        : id{nextID++}, servoMin{servoMin}, servoMax{servoMax}
    {
        this->amplitude = targetAmplitude = amplitude;
        this->offset = targetOffset = offset;
        this->phaseShift = targetPhaseShift = phaseShift;
        this->frequency = targetFrequency = frequency;
    }

    void setPWM(Adafruit_PWMServoDriver &pwm) { this->pwm = &pwm; }

    // Updates the current state of the motor based on the elapsed time (in seconds)
    void update(double dt)
    {
        // Calculate deltas
        float offsetDelta = OFFSET_CHANGE_FACTOR * (targetOffset - offset);
        float amplitudeDelta = AMPLITUDE_CHANGE_FACTOR * (targetAmplitude - amplitude);
        float phaseDelta = (TWO_PI * frequency);
        float phaseShiftDelta = PHASESHIFT_CHANGE_FACTOR * (targetPhaseShift - phaseShift);
        float freqDelta = FREQ_CHANGE_FACTOR * (targetFrequency - frequency);

        // Apply derivative from last tick/frame/timestep
        amplitude += amplitudeDelta * dt;
        phase += phaseDelta * dt;
        offset += offsetDelta * dt;
        phaseShift += phaseShiftDelta * dt;
        frequency += freqDelta * dt;

        position = amplitude * sinf(phase - phaseShift) + offset;

        uint16_t motorAngle = map(constrain(position, 0, 180), 0, 180, servoMin, servoMax);

        pwm->setPWM(id, 0, motorAngle);
    }
    const long tst = map(-90, 0, 180, 100, 450);

    // Adjusts a parameter of this oscillator using information obtained from an OscillatorUpdateCommand
    // Assuming the struct is used as a Serial communication format, one can easily reinterpret a 12byte char[] as an OscillatorUpdateCommand
    void setState(const OscillatorState& command)
    {
        targetFrequency = command.freq;
        targetOffset = command.offset;
        targetAmplitude = command.amp;
        targetPhaseShift = command.phaseShift;
    }

    void printState() const
    {
        Serial.printf("Oscillator %d\n", id);
        Serial.printf("{\n");
        Serial.print("    position: ");
        Serial.println(position);
        Serial.printf("    phase: ");
        Serial.println(phase);
        Serial.printf("    offset: ");
        Serial.println(offset);
        Serial.printf("    amplitude: ");
        Serial.println(amplitude);
        Serial.printf("    phaseShift: ");
        Serial.println(phaseShift);
        Serial.printf("    frequency: ");
        Serial.println(frequency);
        Serial.printf("    servoMin: ");
        Serial.println(servoMin);
        Serial.printf("    servoMax: ");
        Serial.println(servoMax);
        Serial.printf("}\n");
    }

    void printPosition() const { Serial.printf("%d ", position); }

private:
    Adafruit_PWMServoDriver *pwm = nullptr;
    static size_t nextID;

    const float AMPLITUDE_CHANGE_FACTOR = 1;
    const float OFFSET_CHANGE_FACTOR = 0.5;
    const float FREQ_CHANGE_FACTOR = 0.5;
    const float PHASESHIFT_CHANGE_FACTOR = 1;

    const size_t id;

    float phase;

    float amplitude;
    float targetAmplitude;

    float offset = 90;
    float targetOffset = 90;

    float phaseShift;
    float targetPhaseShift;

    float frequency;
    float targetFrequency;

    float position;

    unsigned int servoMin, servoMax;
};

size_t Oscillator::nextID = 0;
