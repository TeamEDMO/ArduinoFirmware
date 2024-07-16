
#include <string>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Oscillator.h"
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <functional>

#include <Adafruit_BNO08x.h>
#include "IMUHelpers.h"

const unsigned int NUM_OSCILLATORS = 8; // this number has to match entries in array osc[] (do NOT modify!!)

// Timing variables
unsigned long lastTime = 0;
unsigned long timeStep = 10; // period used to update CPG state variables and servo motor control (do NOT modify!!)

const float MS_TO_S = 1.0f / 1000.0f;

const std::string idCode{"Athena"};
const std::string hostname{"EDMO: " + idCode};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

Oscillator oscillators[NUM_OSCILLATORS] = {
    Oscillator(100, 454),
    Oscillator(100, 480),
    Oscillator(108, 460),
    Oscillator(100, 454),
    Oscillator(),
    Oscillator(),
    Oscillator(),
    Oscillator()};

const char ssid[] = "Iris";     //  your network SSID (name)
const char pass[] = "edmotest"; // your network password

bool wifiReady = false;

int status = WL_IDLE_STATUS;
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
void setup()
{
    Serial.begin(9600);

    WiFi.setPins(8, 7, 4, 2);
    WiFi.hostname(hostname.c_str());
    WiFi.begin(ssid, pass);

    udp.begin(2121);

    pwm.begin();
    pwm.setPWMFreq(50);
    delay(4);

    for (Oscillator &o : oscillators)
        o.setPWM(pwm);

    pinMode(LED_BUILTIN, OUTPUT);
    analogWrite(LED_BUILTIN, 0);
}

bool tryConnectWiFi()
{
    if (!wifiReady)
    {
        WiFi.begin(ssid, pass);
        wifiReady = true;
        return false;
    }

    switch (WiFi.status())
    {
    case WL_NO_SHIELD:
    case WL_CONNECTED:
        return true;

    case WL_IDLE_STATUS:
        return false;

    default:
        WiFi.begin(ssid, pass);
        return false;
    }
}

bool communicationEstablished()
{
    auto status = WiFi.status();
    return status == WL_CONNECTED || status == WL_NO_SHIELD;
}

IPAddress remoteIP;
uint16_t remotePort;

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
}

void loop()
{
    unsigned long time = millis();
    unsigned long deltaTimeMS = time - lastTime;

    // Reading input can happen regardless of whether CPG updates
    // This ensures that we don't waste time doing nothing and will maximize responsiveness
    readInput();

    // Used as a visual indicator of functionality
    ledControl();

    lastTime = time; // update the previous time step (do NOT modify!!)

    double deltaTimeS = deltaTimeMS * MS_TO_S; // Make an interval with seconds

    for (auto &oscillator : oscillators)
        oscillator.update(deltaTimeS);
}

#pragma region InputHandling
const char commHeader[]{'E', 'D'};
const char commFooter[]{'M', 'O'};

char dataBuffer[512];
size_t dataBufferLength = 0;
bool haveData = false;
bool receivingData = false;

template <typename T>
bool buffcmp(const T *buffer, const T *other, size_t length)
{
    for (int i = 0; i < length; ++i)
    {
        if (buffer[i] != other[i])
            return false;
    }

    return true;
}

bool receivedCommHeader()
{
    const char *lastTwoCharacters = dataBuffer + (dataBufferLength - 2);
    return buffcmp(lastTwoCharacters, commHeader, sizeof(commHeader));
}

bool receivedCommFooter()
{
    const char *lastTwoCharacters = dataBuffer + (dataBufferLength - 2);
    return buffcmp(lastTwoCharacters, commFooter, sizeof(commFooter));
}

// In place removal of escape characters
size_t unescapeBuffer(char *buffer, size_t length)
{
    size_t j = 0; // The length of the buffer with all escape characters removed
    for (int i = 0; i < length; ++i)
    {
        if (i == '\\') // Ignore any escape characters
            ++i;       // And take the next character immediately

        // string ends in an escape character, but there are no characters after
        if (i >= length)
            break;

        buffer[j] = buffer[i];
        ++j;
    }

    return j;
}

void escapeBuffer()
{
    // We are only escaping ED and MO
    // Replacing them with E\D and M\O respectively
    auto &buffer = dataBuffer;
    auto &length = dataBufferLength;

    int countOccurances = 0;

    for (int i = 0; i < dataBufferLength - 1; ++i)
    {
        if (buffcmp(&buffer[i], commHeader, sizeof(commHeader))     // Found header
            || buffcmp(&buffer[i], commHeader, sizeof(commHeader))) // Found footer
            ++countOccurances;
    }

    size_t newLength = length + countOccurances;

    // Escaped string length too large, dropping
    if (newLength > 512)
    {
        length = 0;
        return;
    }

    for (int i = length - 1, j = newLength - 1; i >= 0; --i, --j)
    {
        if (i < length - 2)
        {
            if ((buffer[i] == commHeader[0] && buffer[i + 1] == commHeader[1])     // Found header
                || (buffer[i] == commFooter[0] && buffer[i + 1] == commFooter[1])) // Found footer
            {
                buffer[j] = '\\';
                --j;
            }
        }

        buffer[j] = buffer[i];
    }

    dataBufferLength = newLength;
}

void readInput()
{
    readInputSerial();
    readInputUDP();
}

void readInputUDP()
{
    int packetSize = udp.parsePacket();

    if (packetSize == 0)
        return;

    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    char packetBuffer[packetSize];

    int length = udp.read(packetBuffer, packetSize);

    if (!buffcmp(packetBuffer, commHeader, 2) || !buffcmp(packetBuffer + length - 2, commFooter, 2))
        return;

    if (packetBuffer[2] != 0)
    {
        udp.beginPacket(remoteIP, remotePort);
        udp.write(packetBuffer, packetSize);
        udp.endPacket();
    }

    const auto callback = [](const char *buffer, size_t bufferLength)
    {
        udp.beginPacket(remoteIP, remotePort);
        udp.write((uint8_t)0);
        udp.write(buffer, bufferLength);
        udp.endPacket();
    };

    tryParsePacket(packetBuffer, length, callback);
}

void readInputSerial()
{
    if (!Serial)
        return;

    // Read data if there is anything on the line
    // But we only take at most 1024 bytes at any time to not stall other items on the loop
    //   The excess data will be handled during the next update cycle
    for (int i = 0; i < 1024 && Serial.available(); ++i)
    {
        dataBuffer[dataBufferLength++] = Serial.read();

        // The communication header signals the start of a input/command packet, and we will store future bytes into a buffer for parsing
        //
        // Note that there we are checking for the header, even if the previous packet isn't complete yet
        // This is so that if ever a packet is interrupted, and a footer never arrives
        //    the header of the next message will guarantee that the system recovers from the interrupted packet
        //
        // If the header is part of the data transmitted, it should be escaped using forward slashes
        // i.e "EDMO" -> "\E\D\M\O" or "E\DM\O" (All you have to ensure is that ED and MO isn't adjacent)
        if (dataBufferLength >= 2 && receivedCommHeader())
        {
            receivingData = true;
            dataBufferLength = 2;
            continue;
        }

        // If we aren't actively receiving data, then we actually don't have to do anything with the data that comes in
        if (!receivingData)
        {
            // Make discard every two bytes to ensure we don't overflow the buffer
            // (We need at least two bytes to determine if a header is received)
            if (dataBufferLength > 2)
                dataBufferLength = 0;
            continue;
        }

        // As long as we haven't received the data, we will not proceed with parsing
        if (!receivedCommFooter())
        {
            // SPECIAL CASE: Buffer overflow - Too much data, we drop the transmission
            if (dataBufferLength == 512)
            {
                receivingData = false;
                dataBufferLength = 0;
            }
            continue;
        }

        // Data transmission successful at this point
        // Relinquish control to packet handler
        receivingData = false;

        const auto writeCallback = [](const char *buffer, size_t bufferLength)
        {
            Serial.write(buffer, bufferLength);
        };

        tryParsePacket(dataBuffer, dataBufferLength, writeCallback);

        dataBufferLength = 0;
    }
}

template <typename T>
void tryParsePacket(char *packet, size_t packetSize, T writeCallback)
{
    // Length of the packet without the header/footer
    size_t packetLength = packetSize - 4;

    // Packet without header
    char *packetBuffer = packet + 2;

    // Unescape packet contents in place
    auto ttt = unescapeBuffer(packetBuffer, packetLength);

    char &packetInstruction = packetBuffer[0];
    char *packetData = packetBuffer + 1;

    if (packetInstruction == 0) // Serial peer wants to receive EDMO identifier
    {
        writeCallback(idCode.c_str(), idCode.length());
    }
    else if (packetInstruction == 1) // Serial peer is sending updated oscillator targets
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
    }
}

#pragma endregion