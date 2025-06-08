// ConfigManager.cpp
// Implementation file for Phase 1 Configuration Management

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
        EEPROM.put(EEPROM_VERSION_ADDR, EEPROM_VERSION);
    }
    else
    {
        Serial.println("ConfigManager: Valid EEPROM found, loading configuration");
        loadAll();
    }

    config_loaded = true;
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
    // Network defaults - match Mongoose config
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
    network_config.dhcp_enabled = false;
    strcpy(network_config.hostname, "AiO-NG-v6");

    // GPS defaults
    gps_config.baud_rate = 115200;
    gps_config.sync_enabled = true;
    gps_config.sync_interval = 10;
}

const NetworkConfig &ConfigManager::getNetworkConfig() const
{
    return network_config;
}

void ConfigManager::setNetworkConfig(const NetworkConfig &config)
{
    network_config = config;
    EEPROM.put(NETWORK_CONFIG_ADDR, network_config);
    Serial.println("ConfigManager: Network configuration saved");
}

const GPSConfig &ConfigManager::getGPSConfig() const
{
    return gps_config;
}

void ConfigManager::setGPSConfig(const GPSConfig &config)
{
    gps_config = config;
    EEPROM.put(GPS_CONFIG_ADDR, gps_config);
    Serial.println("ConfigManager: GPS configuration saved");
}

bool ConfigManager::saveAll()
{
    EEPROM.put(NETWORK_CONFIG_ADDR, network_config);
    EEPROM.put(GPS_CONFIG_ADDR, gps_config);
    EEPROM.put(EEPROM_VERSION_ADDR, EEPROM_VERSION);
    Serial.println("ConfigManager: All configurations saved");
    return true;
}

bool ConfigManager::loadAll()
{
    EEPROM.get(NETWORK_CONFIG_ADDR, network_config);
    EEPROM.get(GPS_CONFIG_ADDR, gps_config);
    Serial.println("ConfigManager: All configurations loaded");
    return true;
}

void ConfigManager::resetToDefaults()
{
    Serial.println("ConfigManager: Resetting to factory defaults");
    setDefaults();
    saveAll();
}

bool ConfigManager::isConfigurationValid() const
{
    return config_loaded;
}

void ConfigManager::printConfiguration()
{
    Serial.println("=== Configuration Manager ===");
    Serial.printf("Network IP: %d.%d.%d.%d\n",
                  network_config.ip[0], network_config.ip[1],
                  network_config.ip[2], network_config.ip[3]);
    Serial.printf("Gateway: %d.%d.%d.%d\n",
                  network_config.gateway[0], network_config.gateway[1],
                  network_config.gateway[2], network_config.gateway[3]);
    Serial.printf("DHCP: %s\n", network_config.dhcp_enabled ? "Enabled" : "Disabled");
    Serial.printf("Hostname: %s\n", network_config.hostname);
    Serial.printf("GPS Baud: %lu\n", gps_config.baud_rate);
    Serial.printf("GPS Sync: %s (%d ms)\n",
                  gps_config.sync_enabled ? "Enabled" : "Disabled",
                  gps_config.sync_interval);
    Serial.printf("Config Valid: %s\n", config_loaded ? "Yes" : "No");
    Serial.println("=============================");
}