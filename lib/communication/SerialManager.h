#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "Arduino.h"
#include "../hardware/HardwareManager.h" // Fixed: Use relative path to hardware directory
#include "../system/DiagnosticManager.h" // Fixed: Use relative path to system directory

class SerialManager
{
public:
    SerialManager(HardwareManager &hardwareManager, DiagnosticManager &diagnosticManager);

    bool begin();
    void update();

    // GPS/GNSS communication
    bool sendToGPS(const uint8_t *data, size_t length);
    size_t readFromGPS(uint8_t *buffer, size_t maxLength);

    // RTK Radio communication
    bool sendToRTK(const uint8_t *data, size_t length);
    size_t readFromRTK(uint8_t *buffer, size_t maxLength);

    // ESP32 communication
    bool sendToESP32(const uint8_t *data, size_t length);
    size_t readFromESP32(uint8_t *buffer, size_t maxLength);

    // RS232 communication
    bool sendToRS232(const uint8_t *data, size_t length);
    size_t readFromRS232(uint8_t *buffer, size_t maxLength);

    // Bridge mode handling
    void enableBridgeMode(bool enable);
    bool isBridgeModeEnabled() const;

    // Status
    float getCpuUsage() const;

private:
    HardwareManager &hardwareManager_;
    DiagnosticManager &diagnosticManager_;

    bool bridgeModeEnabled_;
    float serialUsage_;

    // Serial port references (from HardwareManager)
    // These will be initialized in begin() from HardwareManager
};

#endif // SERIAL_MANAGER_H