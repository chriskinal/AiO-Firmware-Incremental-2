#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include "Arduino.h"
#include "../communication/SerialManager.h" // Fixed: Use relative path to communication directory
#include "../system/DiagnosticManager.h"    // Fixed: Use relative path to system directory

// IMU data structure
struct IMUData
{
    float heading = 0.0f;   // Degrees
    float roll = 0.0f;      // Degrees
    float pitch = 0.0f;     // Degrees
    float accelX = 0.0f;    // m/s²
    float accelY = 0.0f;    // m/s²
    float accelZ = 0.0f;    // m/s²
    float gyroX = 0.0f;     // rad/s
    float gyroY = 0.0f;     // rad/s
    float gyroZ = 0.0f;     // rad/s
    uint32_t timestamp = 0; // Milliseconds
    bool isValid = false;
};

class IMUManager
{
public:
    IMUManager(SerialManager &serialManager, DiagnosticManager &diagnosticManager);

    bool begin();
    void update();

    // IMU data access
    const IMUData &getCurrentIMU() const;
    bool hasValidData() const;

    // BNO sensor handling
    bool initializeBNO();
    bool readBNOData();

    // FUSE IMU calculations
    void processFUSEData();

    // Ring buffer management
    void addToRingBuffer(const IMUData &data);
    IMUData getAveragedData(uint8_t samples = 5) const;

    // Heading/roll/pitch processing
    float getFilteredHeading() const;
    float getFilteredRoll() const;
    float getFilteredPitch() const;

    // Status
    float getCpuUsage() const;
    uint32_t getUpdatesProcessed() const;

private:
    SerialManager &serialManager_;
    DiagnosticManager &diagnosticManager_;

    IMUData currentIMU_;
    uint32_t updatesProcessed_;
    float imuUsage_;

    // Ring buffer for averaging
    static constexpr uint8_t RING_BUFFER_SIZE = 16;
    IMUData ringBuffer_[RING_BUFFER_SIZE];
    uint8_t ringBufferIndex_;
    bool ringBufferFull_;

    // Filtering
    float headingFilter_;
    float rollFilter_;
    float pitchFilter_;

    // BNO055 communication
    bool bnoInitialized_;

    // Helper functions
    float normalizeAngle(float angle);
    float applyLowPassFilter(float current, float previous, float alpha = 0.1f);
};

#endif // IMU_MANAGER_H