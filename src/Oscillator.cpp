#include "Oscillator.h"
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

static uint8_t nextID = 0;

Oscillator::Oscillator(unsigned int servoMin, unsigned int servoMax, const OscillatorParams &parameters)
    : id{nextID++}, servoMin{servoMin}, servoMax{servoMax}, state{parameters}, params{parameters}, defaultParams{parameters}
{
}

void Oscillator::setPWM(Adafruit_PWMServoDriver &pwm) { this->pwm = &pwm; }

void Oscillator::update(double dt)
{
    // Calculate deltas
    float offsetDelta = OFFSET_CHANGE_FACTOR * (params.offset - state.offset);
    float amplitudeDelta = AMPLITUDE_CHANGE_FACTOR * (params.amp - state.amp);
    float phaseDelta = (TWO_PI * state.freq);
    float phaseShiftDelta = PHASESHIFT_CHANGE_FACTOR * (params.phaseShift - state.phaseShift);
    float freqDelta = FREQ_CHANGE_FACTOR * (params.freq - state.freq);

    // Apply derivative from last tick/frame/timestep
    state.amp += amplitudeDelta * dt;
    state.phase += phaseDelta * dt;
    state.offset += offsetDelta * dt;
    state.phaseShift += phaseShiftDelta * dt;
    state.freq += freqDelta * dt;

    float position = state.amp * sinf(state.phase - state.phaseShift) + state.offset;

    uint16_t motorAngle = map(constrain(position, 25, 180), 0, 180, servoMin, servoMax);

    pwm->setPWM(id, 0, motorAngle);
}

void Oscillator::reset()
{
    state = params = defaultParams;
}

void Oscillator::setParams(const OscillatorParams &command)
{
    params = command;
}

const OscillatorState &Oscillator::getState() const
{
    return state;
}