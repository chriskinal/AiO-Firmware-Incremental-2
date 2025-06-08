// ConfigManager.h
// Phase 1: Configuration Management Class

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "Arduino.h"
#include "EEPROM.h"

// Network configuration structure
struct NetworkConfig
{
    uint8_t ip[4] = {192, 168, 5, 126};
    uint8_t gateway[4] = {192, 168, 5, 1};
    uint8_t subnet[4] = {255, 255, 255, 0};
    uint8_t dns[4] = {8, 8, 8, 8};
    bool dhcp_enabled = false;
    char hostname[32] = "AiO-NG-v6";
};

// GPS configuration structure
struct GPSConfig
{
    uint32_t baud_rate = 115200;
    bool sync_enabled = true;
    bool pass_through = false;
    uint8_t sync_interval = 10; // ms
    char sync_mode[16] = "10ms-UM98x";
};

// Autosteer configuration structure
struct AutosteerConfig
{
    uint8_t kp = 40;
    uint8_t low_pwm = 10;
    int16_t was_offset = 0;
    uint8_t min_pwm = 9;
    uint8_t high_pwm = 150;
    float sensor_counts = 120.0;
    float ackerman_fix = 1.0;
    bool invert_was = false;
    bool single_input_was = true;
};

// Machine configuration structure
struct MachineConfig
{
    uint8_t section_count = 8;
    bool hydraulic_enabled = false;
    bool tramline_enabled = false;
    uint16_t work_switch_type = 0;
    uint16_t rate_applied_per_hour = 0;
};

class ConfigManager
{
private:
    // EEPROM addresses
    static const uint16_t EEPROM_VERSION_ADDR = 0;
    static const uint16_t NETWORK_CONFIG_ADDR = 100;
    static const uint16_t GPS_CONFIG_ADDR = 200;
    static const uint16_t AUTOSTEER_CONFIG_ADDR = 300;
    static const uint16_t MACHINE_CONFIG_ADDR = 400;

    static const uint16_t EEPROM_VERSION = 2406;

    NetworkConfig network_config;
    GPSConfig gps_config;
    AutosteerConfig autosteer_config;
    MachineConfig machine_config;

    bool config_loaded = false;

    // Helper methods
    bool isValidEEPROM();
    void setDefaults();
    void saveVersion();

public:
    ConfigManager();

    // Initialize and load configuration
    bool begin();

    // Network configuration
    const NetworkConfig &getNetworkConfig() const { return network_config; }
    void setNetworkConfig(const NetworkConfig &config);

    // GPS configuration
    const GPSConfig &getGPSConfig() const { return gps_config; }
    void setGPSConfig(const GPSConfig &config);

    // Autosteer configuration
    const AutosteerConfig &getAutosteerConfig() const { return autosteer_config; }
    void setAutosteerConfig(const AutosteerConfig &config);

    // Machine configuration
    const MachineConfig &getMachineConfig() const { return machine_config; }
    void setMachineConfig(const MachineConfig &config);

    // Save all configurations to EEPROM
    bool saveAll();

    // Load all configurations from EEPROM
    bool loadAll();

    // Reset to factory defaults
    void resetToDefaults();

    // Configuration status
    bool isConfigurationValid() const { return config_loaded; }

    // Print configuration for debugging
    void printConfiguration();
};

#endif // CONFIG_MANAGER_H