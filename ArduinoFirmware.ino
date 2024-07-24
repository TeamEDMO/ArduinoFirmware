
#include "globals.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <cstring>
#include "Oscillator.h"
#include "IMUSensor.h"

#include "Communications/PacketUtils.h"
#include "Communications/WiFiCommStream.h"
#include "Communications/SerialCommStream.h"

#include "TimingUtils.h"

// Timing variables
unsigned long lastTime = 0;
unsigned long timeStep = 10; // period used to update CPG state variables and servo motor control (do NOT modify!!)

const float MS_TO_S = 1.0f / 1000.0f;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup()
{
    SerialComms.init();
    SerialComms.bindPacketHandler(packetHandler);

    WifiComms.init();
    WifiComms.bindPacketHandler(packetHandler);

    imu.init();

    pwm.begin();
    pwm.setPWMFreq(50);
    delay(4);

    for (Oscillator &o : oscillators)
        o.setPWM(pwm);

    pinMode(LED_BUILTIN, OUTPUT);
    analogWrite(LED_BUILTIN, 0);
}

uint8_t ledState = 0;
bool reversing = true;

void ledControl()
{
    const uint8_t fadeSpeed = 4;

    uint8_t newLedState = ledState + (reversing ? -fadeSpeed : fadeSpeed);

    if (newLedState == 0)
    {
        reversing = !reversing;
        return;
    }
    ledState = newLedState;

    analogWrite(LED_BUILTIN, ledState);
    // Serial.println(ledState);
}

void loop()
{
    unsigned long time = millis();
    unsigned long deltaTimeMS = time - lastTime;

    lastTime = time; // update the previous time step (do NOT modify!!)

    // Reading input can happen regardless of whether CPG updates
    // This ensures that we don't waste time doing nothing and will maximize responsiveness
    SerialComms.update();
    WifiComms.update();
    imu.update();

    // Used as a visual indicator of functionality
    ledControl();

    if (deltaTimeMS < 10)
        return;

    double deltaTimeS = deltaTimeMS * MS_TO_S; // Make an interval with seconds

    for (auto &oscillator : oscillators)
        oscillator.update(deltaTimeS);
}

enum PacketInstructions
{
    IDENTIFY = 0,
    SESSION_START = 1,
    GET_TIME = 2,

    UPDATE_OSCILLATOR = 3,
    SEND_MOTOR_DATA = 4,
    SEND_IMU_DATA = 5,
};

// Takes parses a received packet, which contain an instruction and optionally additional data
// This method may write a response via the provided ICommStream pointer
// This method returns true if a response is written, false otherwise.
//
// Any invalid instruction is dropped silently
bool packetHandler(char *packet, size_t packetSize, ICommStream *commStream)
{
    // Length of the packet without the header/footer
    size_t packetLength = packetSize - 4;

    // Packet without header
    char *packetBuffer = packet + 2;

    // Unescape packet contents in place
    auto ttt = unescapeBuffer(packetBuffer, packetLength);

    char &packetInstruction = packetBuffer[0];
    char *packetData = packetBuffer + 1;

    switch (packetInstruction)
    {
    case IDENTIFY:
    {
        commStream->write(commHeader, 2);
        commStream->write(IDENTIFY); // Write the instruction as a response
        commStream->write((uint8_t *)idCode.c_str(), idCode.length());
        commStream->write(commFooter, 2);
        return true;
    }

    case SESSION_START:
    {
        auto currentTime = millis();

        TimingUtils::setReferenceTime(currentTime);

        size_t offsetTime {};
        std::memcpy(&offsetTime, packetData, sizeof(size_t));

        TimingUtils::setOffsetTime(offsetTime);
        return false;
    }

    case GET_TIME:
    {
        auto returnedTime = TimingUtils::getTimeMillis();
        auto dataBytes = reinterpret_cast<char *>(&returnedTime);

        // These data bytes may accidentally contain the header or footer, let's escape it to be safe
        auto adjustedLength = countEscapedLength(dataBytes, 4);
        char escapedData[adjustedLength];

        escapeData(dataBytes, escapedData, adjustedLength);

        commStream->write(commHeader, 2);
        commStream->write(GET_TIME);
        commStream->write(escapedData, adjustedLength);
        commStream->write(commFooter, 2);
        return true;
    }

    case UPDATE_OSCILLATOR:
    {
        OscillatorState updateCommand{};

        std::memcpy(&updateCommand, packetData, sizeof(OscillatorState));
        oscillators[updateCommand.targetOscillatorID].setState(updateCommand);

        return false;
    }

    case SEND_MOTOR_DATA:
    {
        return false;
    }

    case SEND_IMU_DATA:
    {
        commStream->write(commHeader, 2);
        commStream->write(SEND_IMU_DATA);
        imu.printTo(commStream);
        commStream->write(commFooter, 2);
        return true;
    }

    default:
        return false;
    }
}
