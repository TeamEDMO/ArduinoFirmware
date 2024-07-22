#pragma once

#include <Adafruit_BNO08x.h>
#include <cstddef>
#include <array>
#include "Communications/ICommStream.h"

#pragma pack(push, 1)
template <typename T>
struct SensorInfo
{
    uint32_t timestamp;
    uint8_t accuracy;

    T value;
};
#pragma pack(pop)

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

    Adafruit_BNO08x bno08x{};

    bool sensorPresent = false;
    bool initialized = false;
    uint32_t lastUpdateTime = 0;

    struct
    {
        SensorInfo<vec3> gyroscope{}; // We are getting this
        SensorInfo<vec3> accelerometer{};
        SensorInfo<vec3> magneticField{}; // We are getting this
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

        if (bno08x.wasReset())
        {
            //Serial.print("sensor was reset ");
            enableReports();
        }

        sh2_SensorValue_t sensorData;

        // No new sensor data, bailing
        while (bno08x.getSensorEvent(&sensorData))
        {
            switch (sensorData.sensorId)
            {
            case SH2_GYROSCOPE_CALIBRATED:
                rawData.gyroscope = createSensorInfoWith<vec3>(sensorData, sensorData.un.gyroscope);
                break;
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                rawData.magneticField = createSensorInfoWith<vec3>(sensorData, sensorData.un.magneticField);
                break;
            case SH2_LINEAR_ACCELERATION:
                rawData.accelerometer = createSensorInfoWith<vec3>(sensorData, sensorData.un.linearAcceleration);
                break;
            case SH2_ROTATION_VECTOR:
                rawData.rotation = createSensorInfoWith<quarternion>(sensorData, sensorData.un.rotationVector);
                break;
            case SH2_GRAVITY:
                rawData.gravity = createSensorInfoWith<vec3>(sensorData, sensorData.un.gravity);
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
        tryEnable(SH2_LINEAR_ACCELERATION);
        tryEnable(SH2_GYROSCOPE_CALIBRATED);
        tryEnable(SH2_MAGNETIC_FIELD_CALIBRATED);
        tryEnable(SH2_GRAVITY);
        tryEnable(SH2_ROTATION_VECTOR);
    }

    bool enableReport(sh2_SensorId_e sensorID, size_t report_interval = 10000UL)
    {
        static sh2_SensorConfig_t config;

        // These sensor options are disabled or not used in most cases
        config.changeSensitivityEnabled = false;
        config.wakeupEnabled = false;
        config.changeSensitivityRelative = false;
        config.alwaysOnEnabled = false;
        config.changeSensitivity = 0;
        config.batchInterval_us = 200000; // Batch inputs for at most 200ms to not overload CPU
        config.sensorSpecific = 0;

        config.reportInterval_us = report_interval;
        int status = sh2_setSensorConfig(sensorID, &config);

        if (status != SH2_OK)
        {
            return false;
        }

        return true;
    }

    void tryEnable(sh2_SensorId_e reportType)
    {
        enableReport(reportType);
    }

    template <typename T, typename Y>
    SensorInfo<T> createSensorInfoWith(const sh2_SensorValue_t &sensorData, const Y &dataField)
    {
        return SensorInfo<T>{
            sensorData.delay / 1000,
            sensorData.status,
            dataField};
    }
};

IMUSensor imu{};