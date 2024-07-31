#pragma once
#include <cstddef>
// This struct is the format used in serial communication.
// This struct allows users to change any oscillator parameter within 12 bytes
// It also allows us to cast 12 byte char* received from serial communication,
// saving parsing overhead

struct OscillatorParams
{
  const static size_t STRUCT_SIZE;

  float freq;
  float amp;
  float offset;
  float phaseShift;
};

const size_t OscillatorParams::STRUCT_SIZE{sizeof(OscillatorParams)};

struct OscillatorState
{
  const static size_t STRUCT_SIZE;

  float freq;
  float amp;
  float offset;
  float phaseShift;

  float phase;
};

const size_t OscillatorState::STRUCT_SIZE{sizeof(OscillatorState)};
