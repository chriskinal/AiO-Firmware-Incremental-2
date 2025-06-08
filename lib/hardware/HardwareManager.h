// HardwareManager.h
// Phase 1: Hardware Abstraction and Setup

#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

// Pin definitions for Teensy 4.1 AiO-NG-v6
namespace HardwarePins
{
    // Built-in LED
    const uint8_t LED_BUILTIN_PIN = 13;

    // Status LEDs
    const uint8_t STATUS_LED_PIN = 2;

    // Switch inputs
    const uint8_t WORK_SWITCH_PIN = 3;
    const uint8_t STEER_SWITCH_PIN = 4;

    // Analog inputs
    const uint8_t WAS_SENSOR_PIN = A0;
    const uint8_t PRESSURE_SENSOR_PIN = A1;
    const uint8_t CURRENT_SENSOR_PIN = A2;

    // PWM outputs
    const uint8_t MOTOR_PWM_1_PIN = 5;
    const uint8_t MOTOR_PWM_2_PIN = 6;
    const uint8_t MOTOR_DIR_PIN = 7;

    // I2C pins (Wire)
    const uint8_t SDA_PIN = 18;
    const uint8_t SCL_PIN = 19;

    // SPI pins for Ethernet
    const uint8_t MOSI_PIN = 11;
    const uint8_t MISO_PIN = 12;
    const uint8_t SCK_PIN = 13;
    const uint8_t CS_ETHERNET_PIN = 10;

    // Serial ports
    // Serial1: GPS (pins 0,1)
    // Serial2: RTK Radio (pins 7,8)
    // Serial3: ESP32 (pins 15,14)
    // Serial4: RS232 (pins 16,17)
}

class HardwareManager
{
private:
    bool hardware_initialized = false;
    bool i2c_initialized = false;
    bool spi_initialized = false;

    // PWM frequency settings
    uint32_t pwm_frequency = 1000; // 1kHz default

    // I2C settings
    uint32_t i2c_frequency = 400000; // 400kHz

    // Status tracking
    bool led_state = false;
    uint32_t last_heartbeat = 0;

public:
    HardwareManager();

    // Initialize all hardware
    bool begin();

    // Individual hardware initialization
    bool initializePins();
    bool initializeI2C();
    bool initializeSPI();
    bool initializeSerial();
    bool initializePWM();

    // Pin control methods
    void setPin(uint8_t pin, bool state);
    bool readPin(uint8_t pin);
    uint16_t readAnalog(uint8_t pin);
    void setPWM(uint8_t pin, uint16_t value);

    // LED control
    void setBuiltinLED(bool state);
    void setStatusLED(bool state);
    void toggleBuiltinLED();
    void heartbeat(); // Blink pattern for system alive

    // I2C operations
    bool scanI2C();
    bool writeI2C(uint8_t address, uint8_t reg, uint8_t data);
    uint8_t readI2C(uint8_t address, uint8_t reg);

    // PWM configuration
    void setPWMFrequency(uint32_t frequency);
    uint32_t getPWMFrequency() const { return pwm_frequency; }

    // Hardware status
    bool isInitialized() const { return hardware_initialized; }
    bool isI2CReady() const { return i2c_initialized; }
    bool isSPIReady() const { return spi_initialized; }

    // Diagnostics
    void printHardwareStatus();
    void runHardwareTest();

    // Power management
    void enablePeripheral(const char *peripheral);
    void disablePeripheral(const char *peripheral);

    // Reset functions
    void softReset();
    void watchdogReset();
};

#endif // HARDWARE_MANAGER_H