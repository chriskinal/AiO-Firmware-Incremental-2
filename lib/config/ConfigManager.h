// ConfigManager.h
// Header file for Phase 1 Configuration Management

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "Arduino.h"
#include "EEPROM.h"

// Network configuration structure
struct NetworkConfig
{
    uint8_t ip[4];
    uint8_t gateway[4];
    uint8_t subnet[4];
    bool dhcp_enabled;
    char hostname[32];
};

// GPS configuration structure
struct GPSConfig
{
    uint32_t baud_rate;
    bool sync_enabled;
    uint8_t sync_interval;
};

// ConfigManager class declaration
class ConfigManager
{
private:
    // EEPROM addresses and version - these will be defined in .cpp
    static constexpr uint16_t EEPROM_VERSION_ADDR = 0;
    static constexpr uint16_t NETWORK_CONFIG_ADDR = 100;
    static constexpr uint16_t GPS_CONFIG_ADDR = 200;
    static constexpr uint16_t EEPROM_VERSION = 2406;

    NetworkConfig network_config;
    GPSConfig gps_config;
    bool config_loaded;

    // Private helper methods
    bool isValidEEPROM();
    void setDefaults();

public:
    // Constructor
    ConfigManager();

    // Initialization
    bool begin();

    // Network configuration
    const NetworkConfig &getNetworkConfig() const;
    void setNetworkConfig(const NetworkConfig &config);

    // GPS configuration
    const GPSConfig &getGPSConfig() const;
    void setGPSConfig(const GPSConfig &config);

    // Utility methods
    bool saveAll();
    bool loadAll();
    void resetToDefaults();
    bool isConfigurationValid() const;
    void printConfiguration();
};

#endif // CONFIG_MANAGER_H