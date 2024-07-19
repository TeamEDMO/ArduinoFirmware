
#include "globals.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "Oscillator.h"
#include "IMUSensor.h"

#include "Communications/PacketUtils.h"
#include "Communications/WiFiCommStream.h"
#include "Communications/SerialCommStream.h"

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
    UPDATE_OSCILLATOR = 1,

    SEND_MOTOR_DATA = 2,
    SEND_IMU_DATA = 4,
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

    case UPDATE_OSCILLATOR:
    {
        // Technically we shouldn't need to make a copy of the buffer, since nothing is supposed to write to the buffer at this point
        // However there is an issue with the UDP packets that causes the reinterpret_cast to fail
        // Copying the buffer resolves the issue. Performance impact is negligable if the compiler haven't optimized it
        char test[sizeof(OscillatorState)];
        memcpy(test, packetData, OscillatorState::STRUCT_SIZE);

        // Debug: Whether the state buffer is received in full
        // Serial.write(stateBuffer, OscillatorState::STRUCT_SIZE);

        // We receive a buffer of 20 bytes, these are bytes for an OscillatorState
        // We reinterpret the bytes as an OscillatorState, which can then be directly used to update each oscillator
        //
        // This operation is intentionally low level, as reinterpreting raw bytes is significantly faster than parsing ASCII strings
        //  This also allows for more efficient state transmission. However care must be taken in regards to endianess, and struct alignment/packing.
        OscillatorState command = *reinterpret_cast<OscillatorState *>(test);
        oscillators[command.targetOscillatorID].setState(command);
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
