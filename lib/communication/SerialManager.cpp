#include "SerialManager.h"

SerialManager::SerialManager(HardwareManager &hardwareManager, DiagnosticManager &diagnosticManager)
    : hardwareManager_(hardwareManager), diagnosticManager_(diagnosticManager), bridgeModeEnabled_(false), serialUsage_(0.0f)
{
    Serial.print("\r\n- SerialManager constructor (STUB)");
}

bool SerialManager::begin()
{
    Serial.print("\r\n- Serial Manager initialization");
    Serial.print("\r\n  - Serial ports assigned");
    Serial.print("\r\n- Serial Manager initialization complete");
    return true;
}

void SerialManager::update()
{
    // Stub - minimal update
}

// GPS/GNSS communication - stubs
bool SerialManager::sendToGPS(const uint8_t *data, size_t length)
{
    return true; // Stub
}

size_t SerialManager::readFromGPS(uint8_t *buffer, size_t maxLength)
{
    return 0; // Stub - no data
}

// RTK Radio communication - stubs
bool SerialManager::sendToRTK(const uint8_t *data, size_t length)
{
    return true; // Stub
}

size_t SerialManager::readFromRTK(uint8_t *buffer, size_t maxLength)
{
    return 0; // Stub - no data
}

// ESP32 communication - stubs
bool SerialManager::sendToESP32(const uint8_t *data, size_t length)
{
    return true; // Stub
}

size_t SerialManager::readFromESP32(uint8_t *buffer, size_t maxLength)
{
    return 0; // Stub - no data
}

// RS232 communication - stubs
bool SerialManager::sendToRS232(const uint8_t *data, size_t length)
{
    return true; // Stub
}

size_t SerialManager::readFromRS232(uint8_t *buffer, size_t maxLength)
{
    return 0; // Stub - no data
}

// Bridge mode handling - stubs
void SerialManager::enableBridgeMode(bool enable)
{
    bridgeModeEnabled_ = enable;
}

bool SerialManager::isBridgeModeEnabled() const
{
    return bridgeModeEnabled_;
}

// Status
float SerialManager::getCpuUsage() const
{
    return serialUsage_;
}