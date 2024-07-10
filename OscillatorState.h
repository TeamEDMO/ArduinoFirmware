#pragma once
#include <cstddef>
// This struct is the format used in serial communication.
// This struct allows users to change any oscillator parameter within 12 bytes
// It also allows us to cast 12 byte char* received from serial communication,
// saving parsing overhead

struct OscillatorState
{
  const static size_t STRUCT_SIZE;

  unsigned int targetOscillatorID;
  float freq;
  float amp;
  float offset;
  float phaseShift;
};

const size_t OscillatorState::STRUCT_SIZE{sizeof(OscillatorState)};