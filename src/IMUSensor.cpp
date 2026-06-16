
#include "IMUSensor.h"

#include "Communications/PacketUtils.h"
#include "Communications/CommunicationStream.h"
#include "DebugUtils.h"
#include "TimingUtils.h"

template <typename T, typename Y>
SensorInfo<T> createSensorInfoWith(const sh2_SensorValue_t &sensorData, const Y &dataField)
{
    return SensorInfo<T>{
        TimingUtils::getTimeMillis(),
        sensorData.status,
        dataField};
}

IMUSensor::IMUSensor() : bno08x{BNO08X_RESET} {}

void IMUSensor::init()
{
#if IMU_SPI == 1
    sensorPresent = bno08x.begin_SPI(BNO08X_CS, BNO08X_INT);
#else
    sensorPresent = bno08x.begin_I2C();
#endif

    enableReports();
    initialized = true;
}

void IMUSensor::update()
{
    if (!initialized || !sensorPresent)
        return;

    if (bno08x.wasReset())
    {
        Debug::write("Sensor was reset");
        enableReports();
    }

    sh2_SensorValue_t sensorData;
    for (int i = 0; i < 5; ++i)
    {
        if (!bno08x.getSensorEvent(&sensorData))
            break;

        switch (sensorData.sensorId)
        {
        case SH2_GYROSCOPE_CALIBRATED:
            Debug::write("Getting gyro");
            rawData.gyroscope = createSensorInfoWith<vec3>(sensorData, sensorData.un.gyroscope);
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            Debug::write("Getting mag");
            rawData.magneticField = createSensorInfoWith<vec3>(sensorData, sensorData.un.magneticField);
            break;
        case SH2_LINEAR_ACCELERATION:
            Debug::write("Getting linacc");
            rawData.accelerometer = createSensorInfoWith<vec3>(sensorData, sensorData.un.linearAcceleration);
            break;
        case SH2_ROTATION_VECTOR:
            Debug::write("Getting rot");
            rawData.rotation = createSensorInfoWith<quarternion>(sensorData, sensorData.un.rotationVector);
            break;
        case SH2_GRAVITY:
            Debug::write("Getting grav");
            rawData.gravity = createSensorInfoWith<vec3>(sensorData, sensorData.un.gravity);
            break;
        default:
            break;
        }
    }
    Debug::write("Getting sensor data complete\n");
}

void IMUSensor::printTo(CommunicationStream *commStream)
{
    char *dataBytes = reinterpret_cast<char *>(&rawData);

    // These data bytes may accidentally contain the header or footer, let's escape it to be safe
    auto adjustedLength = countEscapedLength(dataBytes, sizeof(rawData));
    char escapedData[adjustedLength];

    escapeData(dataBytes, escapedData, sizeof(rawData));

    commStream->write(escapedData, adjustedLength);
}
void IMUSensor::enableReports()
{
    bno08x.enableReport(SH2_LINEAR_ACCELERATION);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
    bno08x.enableReport(SH2_GRAVITY);
    bno08x.enableReport(SH2_ROTATION_VECTOR);
}

IMUSensor imu{};