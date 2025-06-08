#ifndef GNSS_PROCESSOR_H
#define GNSS_PROCESSOR_H

#include "Arduino.h"
#include "../communication/SerialManager.h" // Fixed: Use relative path to communication directory
#include "../system/DiagnosticManager.h"    // Fixed: Use relative path to system directory

// GNSS fix quality enumeration
enum class GNSSFixQuality
{
    NoFix = 0,
    GPS = 1,
    DGPS = 2,
    PPS = 3,
    RTK = 4,
    FloatRTK = 5,
    Estimated = 6,
    Manual = 7,
    Simulation = 8
};

// GNSS position data structure
struct GNSSPosition
{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    float speed = 0.0f;
    float heading = 0.0f;
    GNSSFixQuality fixQuality = GNSSFixQuality::NoFix;
    uint8_t satelliteCount = 0;
    float hdop = 99.0f;
    uint32_t timestamp = 0;
};

class GNSSProcessor
{
public:
    GNSSProcessor(SerialManager &serialManager, DiagnosticManager &diagnosticManager);

    bool begin();
    void update();

    // Position data
    const GNSSPosition &getCurrentPosition() const;
    bool hasValidFix() const;

    // NMEA processing
    bool processNMEAMessage(const char *message);
    bool processUBXMessage(const uint8_t *data, size_t length);

    // Status
    float getCpuUsage() const;
    uint32_t getMessagesProcessed() const;

private:
    SerialManager &serialManager_;
    DiagnosticManager &diagnosticManager_;

    GNSSPosition currentPosition_;
    uint32_t messagesProcessed_;
    float gnssUsage_;

    // NMEA parsing helpers
    bool parseGGA(const char *message);
    bool parseRMC(const char *message);
    bool parseVTG(const char *message);

    // UBX parsing helpers
    bool parseUBXNavPVT(const uint8_t *data, size_t length);

    // Utility functions
    double parseCoordinate(const char *coord, const char *hemisphere);
    float parseFloat(const char *str);
    int parseInt(const char *str);
};

#endif // GNSS_PROCESSOR_H