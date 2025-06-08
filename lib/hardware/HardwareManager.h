// HardwareManager.h
// Header file for Phase 1 Hardware Abstraction

#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

// Pin definitions for Teensy 4.1 AiO-NG-v6
namespace HardwarePins
{
    // Built-in LED
    constexpr uint8_t LED_BUILTIN_PIN = 13;

    // Status LEDs
    constexpr uint8_t STATUS_LED_PIN = 2;

    // Switch inputs
    constexpr uint8_t WORK_SWITCH_PIN = 3;
    constexpr uint8_t STEER_SWITCH_PIN = 4;

    // Analog inputs
    constexpr uint8_t WAS_SENSOR_PIN = A0;
    constexpr uint8_t PRESSURE_SENSOR_PIN = A1;
    constexpr uint8_t CURRENT_SENSOR_PIN = A2;

    // PWM outputs
    constexpr uint8_t MOTOR_PWM_1_PIN = 5;
    constexpr uint8_t MOTOR_PWM_2_PIN = 6;
    constexpr uint8_t MOTOR_DIR_PIN = 7;

    // I2C pins (Wire)
    constexpr uint8_t SDA_PIN = 18;
    constexpr uint8_t SCL_PIN = 19;

    // SPI pins for Ethernet
    constexpr uint8_t MOSI_PIN = 11;
    constexpr uint8_t MISO_PIN = 12;
    constexpr uint8_t SCK_PIN = 13;
    constexpr uint8_t CS_ETHERNET_PIN = 10;
}

// Hardware status structure
struct HardwareStatus
{
    bool hardware_initialized;
    bool i2c_ready;
    bool spi_ready;
    uint32_t i2c_frequency;
    uint32_t pwm_frequency;
    uint8_t i2c_device_count;
    bool led_state;
};

class HardwareManager
{
private:
    // Configuration
    static constexpr uint32_t DEFAULT_I2C_FREQUENCY = 400000; // 400kHz
    static constexpr uint32_t DEFAULT_PWM_FREQUENCY = 1000;   // 1kHz

    // Status tracking
    bool hardware_initialized;
    bool i2c_initialized;
    bool spi_initialized;
    uint32_t i2c_frequency;
    uint32_t pwm_frequency;
    bool led_state;
    uint32_t last_heartbeat;
    uint8_t i2c_device_count;

    // Private helper methods
    bool initializePins();
    bool initializeI2C();
    bool initializeSPI();
    bool initializeSerial();
    bool initializePWM();

public:
    // Constructor
    HardwareManager();

    // Initialization
    bool begin();

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
    uint32_t getPWMFrequency() const;

    // Hardware status
    bool isInitialized() const;
    bool isI2CReady() const;
    bool isSPIReady() const;
    HardwareStatus getStatus() const;

    // Diagnostics and testing
    void printHardwareStatus();
    void runHardwareTest();
    void testSensors();

    // System control
    void softReset();
};

#endif // HARDWARE_MANAGER_H