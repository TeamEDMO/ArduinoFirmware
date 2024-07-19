#pragma once

#include <Adafruit_BNO08x.h>
#include <cstddef>
#include <array>
#include "Communications/ICommStream.h"

template <typename T>
struct SensorInfo
{
    uint64_t timestamp;
    uint8_t accuracy;

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
    const uint32_t REPORT_INTERVAL = 500000;

    Adafruit_BNO08x bno08x{};

    bool sensorPresent = false;
    bool initialized = false;

    struct
    {
        SensorInfo<vec3> gyroscope{};
        SensorInfo<vec3> accelerometer{};
        SensorInfo<vec3> magneticField{};
        SensorInfo<vec3> gravity{};
        SensorInfo<quarternion> rotation{};
    } rawData;

public:
    void init()
    {
        sensorPresent = bno08x.begin_I2C();

        if (!sensorPresent)
            return;

        enableReports();
        initialized = true;
    }

    void update()
    {
        if (!initialized || !sensorPresent)
            return;

        // Arbitrarily chosen
        const size_t maxSensorEventsPerUpdate = 5;

        for (int i = 0; i < maxSensorEventsPerUpdate; ++i)
        {
            sh2_SensorValue_t sensorData;

            // No new sensor data, bailing
            if (!bno08x.getSensorEvent(&sensorData))
                return;

            switch (sensorData.sensorId)
            {
            case SH2_LINEAR_ACCELERATION:
                rawData.accelerometer = createSensorInfoWith<vec3>(sensorData, sensorData.un.linearAcceleration);
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                rawData.gyroscope = createSensorInfoWith<vec3>(sensorData, sensorData.un.gyroscope);
                break;
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                rawData.magneticField = createSensorInfoWith<vec3>(sensorData, sensorData.un.magneticField);
                break;
            case SH2_GRAVITY:
                rawData.gravity = createSensorInfoWith<vec3>(sensorData, sensorData.un.gravity);
                break;
            case SH2_ROTATION_VECTOR:
                rawData.rotation = createSensorInfoWith<quarternion>(sensorData, sensorData.un.rotationVector);
                break;
            }
        }
    }

    void printTo(ICommStream *commStream)
    {
        char *dataBytes = reinterpret_cast<char *>(&rawData);

        // These data bytes may accidentally contain the header or footer, let's escape it to be safe
        auto adjustedLength = countEscapedLength(dataBytes, sizeof(rawData));
        char escapedData[adjustedLength];

        escapeData(dataBytes, escapedData, sizeof(rawData));

        commStream->write(dataBytes, sizeof(rawData));
    }

private:
    void enableReports()
    {
        // Pure data
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL);
        bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_INTERVAL);
        bno08x.enableReport(SH2_GRAVITY, REPORT_INTERVAL);
        bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL);
    }
    template <typename T, typename Y>
    SensorInfo<T> createSensorInfoWith(const sh2_SensorValue_t &sensorData, const Y &dataField)
    {
        return SensorInfo<T>{
            sensorData.timestamp,
            sensorData.status,
            dataField};
    }
};

IMUSensor imu{};