// ConfigManager.cpp
// Phase 1: Configuration Management Implementation

#include "ConfigManager.h"

ConfigManager::ConfigManager()
{
    config_loaded = false;
}

bool ConfigManager::begin()
{
    Serial.println("ConfigManager: Initializing...");

    if (!isValidEEPROM())
    {
        Serial.println("ConfigManager: Invalid EEPROM, setting defaults");
        setDefaults();
        saveAll();
        saveVersion();
    }
    else
    {
        Serial.println("ConfigManager: Valid EEPROM found, loading configuration");
        loadAll();
    }

    config_loaded = true;
    printConfiguration();
    return true;
}

bool ConfigManager::isValidEEPROM()
{
    uint16_t stored_version;
    EEPROM.get(EEPROM_VERSION_ADDR, stored_version);
    return (stored_version == EEPROM_VERSION);
}

void ConfigManager::setDefaults()
{
    Serial.println("ConfigManager: Setting default configuration");

    // Network defaults
    network_config.ip[0] = 192;
    network_config.ip[1] = 168;
    network_config.ip[2] = 5;
    network_config.ip[3] = 126;
    network_config.gateway[0] = 192;
    network_config.gateway[1] = 168;
    network_config.gateway[2] = 5;
    network_config.gateway[3] = 1;
    network_config.subnet[0] = 255;
    network_config.subnet[1] = 255;
    network_config.subnet[2] = 255;
    network_config.subnet[3] = 0;
    network_config.dns[0] = 8;
    network_config.dns[1] = 8;
    network_config.dns[2] = 8;
    network_config.dns[3] = 8;
    network_config.dhcp_enabled = false;
    strcpy(network_config.hostname, "AiO-NG-v6");

    // GPS defaults
    gps_config.baud_rate = 115200;
    gps_config.sync_enabled = true;
    gps_config.pass_through = false;
    gps_config.sync_interval = 10;
    strcpy(gps_config.sync_mode, "10ms-UM98x");

    // Autosteer defaults
    autosteer_config.kp = 40;
    autosteer_config.low_pwm = 10;
    autosteer_config.was_offset = 0;
    autosteer_config.min_pwm = 9;
    autosteer_config.high_pwm = 150;
    autosteer_config.sensor_counts = 120.0;
    autosteer_config.ackerman_fix = 1.0;
    autosteer_config.invert_was = false;
    autosteer_config.single_input_was = true;

    // Machine defaults
    machine_config.section_count = 8;
    machine_config.hydraulic_enabled = false;
    machine_config.tramline_enabled = false;
    machine_config.work_switch_type = 0;
    machine_config.rate_applied_per_hour = 0;
}

void ConfigManager::saveVersion()
{
    EEPROM.put(EEPROM_VERSION_ADDR, EEPROM_VERSION);
}

void ConfigManager::setNetworkConfig(const NetworkConfig &config)
{
    network_config = config;
    EEPROM.put(NETWORK_CONFIG_ADDR, network_config);
    Serial.println("ConfigManager: Network configuration saved");
}

void ConfigManager::setGPSConfig(const GPSConfig &config)
{
    gps_config = config;
    EEPROM.put(GPS_CONFIG_ADDR, gps_config);
    Serial.println("ConfigManager: GPS configuration saved");
}

void ConfigManager::setAutosteerConfig(const AutosteerConfig &config)
{
    autosteer_config = config;
    EEPROM.put(AUTOSTEER_CONFIG_ADDR, autosteer_config);
    Serial.println("ConfigManager: Autosteer configuration saved");
}

void ConfigManager::setMachineConfig(const MachineConfig &config)
{
    machine_config = config;
    EEPROM.put(MACHINE_CONFIG_ADDR, machine_config);
    Serial.println("ConfigManager: Machine configuration saved");
}

bool ConfigManager::saveAll()
{
    Serial.println("ConfigManager: Saving all configurations...");

    EEPROM.put(NETWORK_CONFIG_ADDR, network_config);
    EEPROM.put(GPS_CONFIG_ADDR, gps_config);
    EEPROM.put(AUTOSTEER_CONFIG_ADDR, autosteer_config);
    EEPROM.put(MACHINE_CONFIG_ADDR, machine_config);
    saveVersion();

    Serial.println("ConfigManager: All configurations saved");
    return true;
}

bool ConfigManager::loadAll()
{
    Serial.println("ConfigManager: Loading all configurations...");

    EEPROM.get(NETWORK_CONFIG_ADDR, network_config);
    EEPROM.get(GPS_CONFIG_ADDR, gps_config);
    EEPROM.get(AUTOSTEER_CONFIG_ADDR, autosteer_config);
    EEPROM.get(MACHINE_CONFIG_ADDR, machine_config);

    Serial.println("ConfigManager: All configurations loaded");
    return true;
}

void ConfigManager::resetToDefaults()
{
    Serial.println("ConfigManager: Resetting to factory defaults");
    setDefaults();
    saveAll();
    config_loaded = true;
}

void ConfigManager::printConfiguration()
{
    Serial.println("=== Configuration Manager Status ===");
    Serial.printf("Network IP: %d.%d.%d.%d\n",
                  network_config.ip[0], network_config.ip[1],
                  network_config.ip[2], network_config.ip[3]);
    Serial.printf("Gateway: %d.%d.%d.%d\n",
                  network_config.gateway[0], network_config.gateway[1],
                  network_config.gateway[2], network_config.gateway[3]);
    Serial.printf("DHCP: %s\n", network_config.dhcp_enabled ? "Enabled" : "Disabled");
    Serial.printf("Hostname: %s\n", network_config.hostname);
    Serial.printf("GPS Baud: %lu\n", gps_config.baud_rate);
    Serial.printf("GPS Sync: %s\n", gps_config.sync_enabled ? "Enabled" : "Disabled");
    Serial.printf("GPS Mode: %s\n", gps_config.sync_mode);
    Serial.printf("Autosteer Kp: %d\n", autosteer_config.kp);
    Serial.printf("WAS Offset: %d\n", autosteer_config.was_offset);
    Serial.printf("Sensor Counts: %.1f\n", autosteer_config.sensor_counts);
    Serial.printf("Machine Sections: %d\n", machine_config.section_count);
    Serial.printf("Configuration Valid: %s\n", config_loaded ? "Yes" : "No");
    Serial.println("====================================");
}