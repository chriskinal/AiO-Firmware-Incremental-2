// HardwareManager.cpp
// Phase 1: Hardware Abstraction Implementation

#include "HardwareManager.h"

HardwareManager::HardwareManager()
{
    hardware_initialized = false;
    i2c_initialized = false;
    spi_initialized = false;
    led_state = false;
    last_heartbeat = 0;
}

bool HardwareManager::begin()
{
    Serial.println("HardwareManager: Initializing hardware...");

    bool success = true;

    // Initialize in order of dependency
    success &= initializePins();
    success &= initializeI2C();
    success &= initializeSPI();
    success &= initializeSerial();
    success &= initializePWM();

    if (success)
    {
        hardware_initialized = true;
        Serial.println("HardwareManager: All hardware initialized successfully");

        // Run initial hardware test
        runHardwareTest();

        // Initial LED indication
        setBuiltinLED(true);
        delay(100);
        setBuiltinLED(false);
    }
    else
    {
        Serial.println("HardwareManager: Hardware initialization failed!");
    }

    printHardwareStatus();
    return success;
}

bool HardwareManager::initializePins()
{
    Serial.println("HardwareManager: Configuring pins...");

    // LED outputs
    pinMode(HardwarePins::LED_BUILTIN_PIN, OUTPUT);
    pinMode(HardwarePins::STATUS_LED_PIN, OUTPUT);

    // Switch inputs with pullups
    pinMode(HardwarePins::WORK_SWITCH_PIN, INPUT_PULLUP);
    pinMode(HardwarePins::STEER_SWITCH_PIN, INPUT_PULLUP);

    // Motor control outputs
    pinMode(HardwarePins::MOTOR_PWM_1_PIN, OUTPUT);
    pinMode(HardwarePins::MOTOR_PWM_2_PIN, OUTPUT);
    pinMode(HardwarePins::MOTOR_DIR_PIN, OUTPUT);

    // Analog inputs (automatic on Teensy)
    analogReadResolution(12); // 12-bit ADC resolution
    analogReference(DEFAULT); // 3.3V reference

    Serial.println("HardwareManager: Pin configuration complete");
    return true;
}

bool HardwareManager::initializeI2C()
{
    Serial.println("HardwareManager: Initializing I2C...");

    Wire.begin();
    Wire.setClock(i2c_frequency);
    Wire.setTimeout(1000); // 1 second timeout

    // Test I2C bus
    if (scanI2C())
    {
        i2c_initialized = true;
        Serial.printf("HardwareManager: I2C initialized at %lu Hz\n", i2c_frequency);
    }
    else
    {
        Serial.println("HardwareManager: I2C initialization failed or no devices found");
        i2c_initialized = false;
    }

    return i2c_initialized;
}

bool HardwareManager::initializeSPI()
{
    Serial.println("HardwareManager: Initializing SPI...");

    SPI.begin();
    SPI.setMOSI(HardwarePins::MOSI_PIN);
    SPI.setMISO(HardwarePins::MISO_PIN);
    SPI.setSCK(HardwarePins::SCK_PIN);

    // Configure Ethernet CS pin
    pinMode(HardwarePins::CS_ETHERNET_PIN, OUTPUT);
    digitalWrite(HardwarePins::CS_ETHERNET_PIN, HIGH);

    spi_initialized = true;
    Serial.println("HardwareManager: SPI initialized successfully");
    return true;
}

bool HardwareManager::initializeSerial()
{
    Serial.println("HardwareManager: Initializing serial ports...");

    // USB Serial already initialized in main

    // GPS Serial (Serial1)
    Serial1.begin(115200);
    Serial.println("HardwareManager: GPS Serial (Serial1) initialized at 115200");

    // RTK Radio Serial (Serial2)
    Serial2.begin(115200);
    Serial.println("HardwareManager: RTK Radio Serial (Serial2) initialized at 115200");

    // ESP32 Serial (Serial3)
    Serial3.begin(115200);
    Serial.println("HardwareManager: ESP32 Serial (Serial3) initialized at 115200");

    // RS232 Serial (Serial4)
    Serial4.begin(115200);
    Serial.println("HardwareManager: RS232 Serial (Serial4) initialized at 115200");

    return true;
}

bool HardwareManager::initializePWM()
{
    Serial.println("HardwareManager: Initializing PWM...");

    // Set PWM frequency for motor control pins
    analogWriteFrequency(HardwarePins::MOTOR_PWM_1_PIN, pwm_frequency);
    analogWriteFrequency(HardwarePins::MOTOR_PWM_2_PIN, pwm_frequency);

    // Initialize PWM outputs to zero
    analogWrite(HardwarePins::MOTOR_PWM_1_PIN, 0);
    analogWrite(HardwarePins::MOTOR_PWM_2_PIN, 0);
    digitalWrite(HardwarePins::MOTOR_DIR_PIN, LOW);

    Serial.printf("HardwareManager: PWM initialized at %lu Hz\n", pwm_frequency);
    return true;
}

void HardwareManager::setPin(uint8_t pin, bool state)
{
    digitalWrite(pin, state ? HIGH : LOW);
}

bool HardwareManager::readPin(uint8_t pin)
{
    return digitalRead(pin) == HIGH;
}

uint16_t HardwareManager::readAnalog(uint8_t pin)
{
    return analogRead(pin);
}

void HardwareManager::setPWM(uint8_t pin, uint16_t value)
{
    // Teensy 4.1 has 12-bit PWM resolution (0-4095)
    uint16_t pwm_value = constrain(value, 0, 4095);
    analogWrite(pin, pwm_value);
}

void HardwareManager::setBuiltinLED(bool state)
{
    led_state = state;
    digitalWrite(HardwarePins::LED_BUILTIN_PIN, state ? HIGH : LOW);
}

void HardwareManager::setStatusLED(bool state)
{
    digitalWrite(HardwarePins::STATUS_LED_PIN, state ? HIGH : LOW);
}

void HardwareManager::toggleBuiltinLED()
{
    led_state = !led_state;
    setBuiltinLED(led_state);
}

void HardwareManager::heartbeat()
{
    uint32_t now = millis();
    if (now - last_heartbeat >= 1000)
    { // 1 second heartbeat
        toggleBuiltinLED();
        last_heartbeat = now;
    }
}

bool HardwareManager::scanI2C()
{
    Serial.println("HardwareManager: Scanning I2C bus...");

    int device_count = 0;
    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf("HardwareManager: I2C device found at address 0x%02X\n", address);
            device_count++;
        }
    }

    if (device_count == 0)
    {
        Serial.println("HardwareManager: No I2C devices found");
        return false;
    }
    else
    {
        Serial.printf("HardwareManager: Found %d I2C device(s)\n", device_count);
        return true;
    }
}

bool HardwareManager::writeI2C(uint8_t address, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

uint8_t HardwareManager::readI2C(uint8_t address, uint8_t reg)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission() != 0)
        return 0xFF;

    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available())
    {
        return Wire.read();
    }
    return 0xFF;
}

void HardwareManager::setPWMFrequency(uint32_t frequency)
{
    pwm_frequency = frequency;
    analogWriteFrequency(HardwarePins::MOTOR_PWM_1_PIN, frequency);
    analogWriteFrequency(HardwarePins::MOTOR_PWM_2_PIN, frequency);
    Serial.printf("HardwareManager: PWM frequency set to %lu Hz\n", frequency);
}

void HardwareManager::printHardwareStatus()
{
    Serial.println("=== Hardware Manager Status ===");
    Serial.printf("Hardware Initialized: %s\n", hardware_initialized ? "Yes" : "No");
    Serial.printf("I2C Ready: %s\n", i2c_initialized ? "Yes" : "No");
    Serial.printf("SPI Ready: %s\n", spi_initialized ? "Yes" : "No");
    Serial.printf("I2C Frequency: %lu Hz\n", i2c_frequency);
    Serial.printf("PWM Frequency: %lu Hz\n", pwm_frequency);
    Serial.printf("Built-in LED: %s\n", led_state ? "ON" : "OFF");
    Serial.printf("Free RAM: %lu bytes\n", (unsigned long)(1024 * 1024 - (uint32_t)__brkval));
    Serial.println("==============================");
}

void HardwareManager::runHardwareTest()
{
    Serial.println("HardwareManager: Running hardware test...");

    // Test LEDs
    Serial.println("Testing LEDs...");
    setBuiltinLED(true);
    setStatusLED(true);
    delay(200);
    setBuiltinLED(false);
    setStatusLED(false);

    // Test analog inputs
    Serial.println("Testing analog inputs...");
    uint16_t was_reading = readAnalog(HardwarePins::WAS_SENSOR_PIN);
    uint16_t pressure_reading = readAnalog(HardwarePins::PRESSURE_SENSOR_PIN);
    uint16_t current_reading = readAnalog(HardwarePins::CURRENT_SENSOR_PIN);

    Serial.printf("WAS Sensor: %d\n", was_reading);
    Serial.printf("Pressure Sensor: %d\n", pressure_reading);
    Serial.printf("Current Sensor: %d\n", current_reading);

    // Test switch inputs
    Serial.println("Testing switch inputs...");
    bool work_switch = readPin(HardwarePins::WORK_SWITCH_PIN);
    bool steer_switch = readPin(HardwarePins::STEER_SWITCH_PIN);

    Serial.printf("Work Switch: %s\n", work_switch ? "PRESSED" : "OPEN");
    Serial.printf("Steer Switch: %s\n", steer_switch ? "PRESSED" : "OPEN");

    Serial.println("HardwareManager: Hardware test complete");
}

void HardwareManager::enablePeripheral(const char *peripheral)
{
    Serial.printf("HardwareManager: Enabling %s\n", peripheral);
    // Implementation depends on specific peripheral
}

void HardwareManager::disablePeripheral(const char *peripheral)
{
    Serial.printf("HardwareManager: Disabling %s\n", peripheral);
    // Implementation depends on specific peripheral
}

void HardwareManager::softReset()
{
    Serial.println("HardwareManager: Performing soft reset...");
    delay(100);
    SCB_AIRCR = 0x05FA0004; // Teensy 4.x software reset
}

void HardwareManager::watchdogReset()
{
    Serial.println("HardwareManager: Watchdog reset requested");
    // Teensy 4.x watchdog implementation would go here
}