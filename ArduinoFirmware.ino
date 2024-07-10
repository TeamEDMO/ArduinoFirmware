
#include <string>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Oscillator.h"

const unsigned int NUM_OSCILLATORS = 8; // this number has to match entries in array osc[] (do NOT modify!!)

// Timing variables
unsigned long lastTime = 0;
unsigned long timeStep = 10; // period used to update CPG state variables and servo motor control (do NOT modify!!)

const float MS_TO_S = 1.0f / 1000.0f;

const std::string idCode{"Bloom"};

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

void setup()
{
    Serial.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(50);
    delay(4);

    for (Oscillator &o : oscillators)
        o.setPWM(pwm);
}

void loop()
{
    unsigned long time = millis();
    unsigned long deltaTimeMS = time - lastTime;

    // Reading input can happen regardless of whether CPG updates
    // This ensures that we don't waste time doing nothing and will maximize responsiveness
    readInput();

    if (deltaTimeMS < timeStep) // CPG update interval (10ms)(do NOT modify!!)
        return;

    lastTime = time; // update the previous time step (do NOT modify!!)

    double deltaTimeS = deltaTimeMS * MS_TO_S; // Make an interval with seconds

    for (auto &oscillator : oscillators)
        oscillator.update(deltaTimeS);
}

#pragma region InputHandling
const char commHeader[2]{'E', 'D'};
const char commFooter[2]{'M', 'O'};

char dataBuffer[512];
size_t dataBufferLength = 0;
bool receivingData = false;

bool receivedCommHeader()
{
    if (dataBufferLength < 2)
        return false;

    char *lastTwoCharacters = dataBuffer + (dataBufferLength - 2);
    
    return (lastTwoCharacters[0] == commHeader[0]) && (lastTwoCharacters[1] == commHeader[1]);
}

bool receivedCommFooter()
{
    char *lastTwoCharacters = dataBuffer + (dataBufferLength - 2);

    return (lastTwoCharacters[0] == commFooter[0]) && (lastTwoCharacters[1] == commFooter[1]);
}

void unescapeBuffer()
{
    size_t j = 0; // The length of the buffer with all escape characters removed
    for (int i = 0; i < dataBufferLength; ++i)
    {
        if (i == '\\') // Ignore any escape characters
            ++i;       // And take the next character immediately

        // string ends in an escape character, but there are no characters after
        if (i >= dataBufferLength)
            break;

        dataBuffer[j] = dataBuffer[i];
        ++j;
    }

    dataBufferLength = j;
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
        if ((buffer[i] == commHeader[0] && buffer[i + 1] == commHeader[1])     // Found header
            || (buffer[i] == commFooter[0] && buffer[i + 1] == commFooter[1])) // Found footer
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
    // Read data if there is anything on the line
    // But we only take at most 1024 bytes at any time to not stall other items on the loop
    //   The excess data will
    for (int i = 0; i < 1024 && Serial.available(); ++i)
    {
        dataBuffer[dataBufferLength++] = Serial.read();

        // We drop everything to receive new packet
        if (dataBufferLength >= 2 && receivedCommHeader())
        {
            //Serial.write("Received comm header");
            receivingData = true;
            dataBufferLength = 0;
            continue;
        }

        // If we aren't actively receiving data, then we actually don't have to do anything.
        if (!receivingData)
        {
            // Make discard every two bytes to ensure we don't overflow the buffer
            if (dataBufferLength > 2)
                dataBufferLength = 0;
            continue;
        }

        if (!receivedCommFooter())
        {
            // Buffer overflow: Too much data, we drop the transmission
            if (dataBufferLength == 512)
            {
                receivingData = false;
                dataBufferLength = 0;
            }
            continue;
        }

        receivingData = false;

        // Data transmission sucessful at this point
        // Remove the footer from the data range
        dataBufferLength -= 2;

        // Remove escape bytes
        unescapeBuffer();

        // Serial peer wants to collect state
        if (dataBuffer[0] == 0)
        {
            Serial.write(idCode.c_str());
        }
        else if (dataBuffer[0] == 1) // Serial peer is sending updated targets
        {
            // State starts at index 1
            char *stateBuffer = &dataBuffer[1];

            Serial.write(stateBuffer, OscillatorState::STRUCT_SIZE);

            // We receive a buffer of chars, these are bytes for an OsciallatorUpdateCommand
            // We reinterpret the bytes as an OsciallatorUpdateCommand
            OscillatorState command = *reinterpret_cast<OscillatorState *>(stateBuffer);

            oscillators[command.targetOscillatorID].setState(command);
        }

        dataBufferLength = 0;
    }
}
#pragma endregion