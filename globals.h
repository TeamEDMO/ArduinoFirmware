/*
 * This file contains variables and defines related to the configuration of the robot.
 */

#pragma once
#include <string>
#include "Oscillator.h"

// Name of the device
const std::string idCode{"Athena"};

// Packet headers and footers
const char commHeader[]{'E', 'D'};
const char commFooter[]{'M', 'O'};

// WiFi support stuff

#define WIFI_SUPPORT 1

#if WIFI_SUPPORT == 1
const std::string hostname{"EDMO: " + idCode};
const char ssid[]{"Iris"};     //  your network SSID (name)
const char pass[]{"edmotest"}; // your network password
#endif

// Oscilator specifications
const unsigned int NUM_OSCILLATORS = 8; // this number has to match entries in array osc[] (do NOT modify!!)
Oscillator oscillators[NUM_OSCILLATORS] = {
    Oscillator(100, 454),
    Oscillator(100, 480),
    Oscillator(108, 460),
    Oscillator(100, 454),
    Oscillator(),
    Oscillator(),
    Oscillator(),
    Oscillator()};
