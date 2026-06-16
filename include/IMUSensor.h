#pragma once

#include <Adafruit_BNO08x.h>
#include <cstddef>
#include "globals.h"

#if IMU_SPI == 1
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET 5
#else
#define BNO08X_RESET -1
#endif

class CommunicationStream;

template <typename T>
struct SensorInfo
{
    uint32_t timestamp;
    uint8_t accuracy;
    // There is 3 padding bytes here

    T value;
};

typedef struct vec3
{
    vec3()
    {
        x = y = z = 0;
    }

    // Conversion template for similar looking structs
    template <typename T>
    vec3(const T &t)
    {
        x = t.x;
        y = t.y;
        z = t.z;
    }

    float x, y, z;
} vec3;

typedef struct quarternion
{
    quarternion()
    {
        i = j = k = real = 0;
    }

    // Conversion template for similar looking structs
    template <typename T>
    quarternion(const T &t)
    {
        i = t.i;
        j = t.j;
        k = t.k;
        real = t.real;
    }

    float i, j, k, real;
} quarternion;

class IMUSensor
{
private:
    // This is measured in microseconds (1000µs == 1ms)
    // Smaller intervals => More sensor data, but may take up too much CPU time to process
    //  The default here is set to 500ms
    const uint32_t REPORT_INTERVAL = 1000;

    Adafruit_BNO08x bno08x;

    bool sensorPresent;
    bool initialized;
    uint32_t lastUpdateTime;

public:
    IMUSensor();

    struct IMUData
    {
        SensorInfo<vec3> gyroscope{}; // We are getting this
        SensorInfo<vec3> accelerometer{};
        SensorInfo<vec3> magneticField{}; // We are getting this
        SensorInfo<vec3> gravity{};
        SensorInfo<quarternion> rotation{};
    } rawData;

    void init();
    void update();
    void printTo(CommunicationStream *commStream);

private:
    void enableReports();
};

extern IMUSensor imu;