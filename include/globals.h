/*
 * This file contains variables and defines related to the configuration of the robot.
 */

#pragma once
#include <string>
#include <tuple>

// Name of the device
const std::string idCode{"Suzanne"};

// Packet headers and footers
const char commHeader[]{'E', 'D'};
const char commFooter[]{'M', 'O'};

// WiFi support stuff

#define WIFI_SUPPORT 1

#if WIFI_SUPPORT == 1
const std::string hostname{"EDMO: " + idCode};
const char ssid[]{"Asteria"};  //  your network SSID (name)
const char pass[]{"asteria1"}; // your network password
#endif

// Oscilator specifications
const std::tuple<unsigned int, unsigned int> oscillatorLimits[]{
    {100, 600},
    {100, 600},
    {100, 600},
    {100, 600},
};
const uint16_t NUM_OSCILLATORS = sizeof(oscillatorLimits) / sizeof(oscillatorLimits[0]); // this number has to match entries in array osc[] (do NOT modify!!)

const uint16_t oscillatorColours[NUM_OSCILLATORS]{
    0, 120, 240, 60};

// SPI has faster throughput, but more wires
#define IMU_SPI 1