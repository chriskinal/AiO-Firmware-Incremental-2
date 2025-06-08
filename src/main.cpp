// main.cpp
// Phase 1 Refactored: Based on original working Mongoose code

#include "Arduino.h"
#include "mongooseStart.h"
#include "ConfigManager.h"
#include "HardwareManager.h"

// Global instances
ConfigManager configManager;
HardwareManager hardwareManager;

// Phase 1 placeholders - will be replaced with actual classes later
bool phase1_initialized = false;

// Timing variables
unsigned long heartbeatTimer = 0;
unsigned long diagnosticTimer = 0;

// Function declarations
void initializePhase1();
void printSystemDiagnostics();
void handleDebugCommands();
void printHelpMenu();
uint32_t freeMemory();

void setup()
{
  // Keep original working setup but add Phase 1 framework
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    delay(50);

  Serial.println("=== AiO-NG-v6 Phase 1 Startup ===");

  // Phase 1: Initialize framework
  initializePhase1();

  // Original working Mongoose initialization
  ethernet_init();
  mongoose_init();

  Serial.println("=== Phase 1 System Ready ===");
  Serial.println("Type 'help' for debug commands");
}

void loop()
{
  // Keep original working loop but add Phase 1 features
  mongoose_poll();

  unsigned long now = millis();

  // Simple heartbeat
  if (now - heartbeatTimer >= 1000)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    heartbeatTimer = now;
  }

  // Periodic diagnostics
  if (now - diagnosticTimer >= 10000)
  { // Every 10 seconds
    printSystemDiagnostics();
    diagnosticTimer = now;
  }

  // Handle serial debug commands
  if (Serial.available())
  {
    handleDebugCommands();
  }
}

void initializePhase1()
{
  Serial.println("Phase 1: Initializing Core Infrastructure...");

  // Initialize real ConfigManager
  if (!configManager.begin())
  {
    Serial.println("ERROR: ConfigManager initialization failed!");
    phase1_initialized = false;
    return;
  }

  // Show loaded configuration
  const NetworkConfig &netConfig = configManager.getNetworkConfig();
  Serial.printf("ConfigManager: Loaded IP %d.%d.%d.%d\n",
                netConfig.ip[0], netConfig.ip[1],
                netConfig.ip[2], netConfig.ip[3]);

  // Initialize real HardwareManager
  if (!hardwareManager.begin())
  {
    Serial.println("ERROR: HardwareManager initialization failed!");
    phase1_initialized = false;
    return;
  }

  // DiagnosticManager still placeholder
  Serial.println("- DiagnosticManager: Ready (placeholder)");

  phase1_initialized = true;
  Serial.println("Phase 1: Core Infrastructure ready");
}

void printSystemDiagnostics()
{
  Serial.println("=== System Diagnostics ===");
  Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
  Serial.printf("Free RAM: %lu bytes\n", freeMemory());
  Serial.printf("Phase 1 Status: %s\n", phase1_initialized ? "Active" : "Inactive");
  Serial.printf("Mongoose: Active\n");
  Serial.println("========================");
}

void handleDebugCommands()
{
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "status")
  {
    printSystemDiagnostics();
  }
  else if (command == "test")
  {
    Serial.println("Running Phase 1 test...");

    // Test LED
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
    Serial.println("LED test complete");
  }
  else if (command == "config")
  {
    configManager.printConfiguration();
  }
  else if (command == "hardware")
  {
    hardwareManager.printHardwareStatus();
  }
  else if (command == "hwtest")
  {
    hardwareManager.runHardwareTest();
  }
  else if (command == "sensors")
  {
    hardwareManager.testSensors();
  }
  else if (command == "scan")
  {
    hardwareManager.scanI2C();
    hardwareManager.scanI2C();
  }
  else if (command == "defaults")
  {
    Serial.println("Resetting configuration to defaults...");
    configManager.resetToDefaults();
    Serial.println("Configuration reset complete. System will use new settings on next boot.");
  }
  else if (command == "save")
  {
    configManager.saveAll();
    Serial.println("Configuration saved to EEPROM");
  }
  else if (command == "phase1")
  {
    Serial.printf("Phase 1 Status: %s\n", phase1_initialized ? "Active" : "Inactive");
    Serial.println("Phase 1 includes: ConfigManager, HardwareManager, DiagnosticManager");
  }
  else if (command == "reset")
  {
    Serial.println("Performing system reset...");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; // Teensy 4.x software reset
  }
  else if (command == "help")
  {
    printHelpMenu();
  }
  else if (command.length() > 0)
  {
    Serial.printf("Unknown command: %s\n", command.c_str());
    Serial.println("Type 'help' for available commands");
  }
}

void printHelpMenu()
{
  Serial.println("=== Phase 1 Debug Commands ===");
  Serial.println("status   - Show system status");
  Serial.println("config   - Show configuration");
  Serial.println("hardware - Show hardware status");
  Serial.println("hwtest   - Run safe hardware test");
  Serial.println("sensors  - Test analog sensors (may reset)");
  Serial.println("scan     - Scan I2C bus");
  Serial.println("defaults - Reset config to defaults");
  Serial.println("save     - Save config to EEPROM");
  Serial.println("test     - Run basic test");
  Serial.println("phase1   - Show Phase 1 status");
  Serial.println("reset    - Reset system");
  Serial.println("help     - Show this menu");
  Serial.println("==============================");
}

// Memory usage helper - Teensy 4.1 using linker symbols
extern "C"
{
  extern char _ebss;   // End of BSS section
  extern char _estack; // End of stack
}

uint32_t freeMemory()
{
  // Use actual linker symbols for Teensy 4.1
  char stack_top;

  // Calculate free memory as difference between current stack and heap
  return &stack_top - &_ebss;
}