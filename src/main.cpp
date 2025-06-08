// main.cpp
// Phase 1 Complete: ConfigManager, HardwareManager, DiagnosticManager

#include "Arduino.h"
#include "mongooseStart.h"
#include "ConfigManager.h"
#include "HardwareManager.h"
#include "DiagnosticManager.h"

// Global instances - Phase 1 Complete
ConfigManager configManager;
HardwareManager hardwareManager;
DiagnosticManager diagnosticManager;

// Phase 1 status
bool phase1_initialized = false;

// Timing variables
unsigned long heartbeatTimer = 0;
unsigned long diagnosticTimer = 0;

// Function declarations
void initializePhase1();
void handleDebugCommands();

void setup()
{
  // Keep original working setup but add complete Phase 1 framework
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    delay(50);

  Serial.println("=== AiO-NG-v6 Phase 1 Complete Startup ===");

  // Phase 1: Initialize all core infrastructure classes
  initializePhase1();

  // Original working Mongoose initialization
  ethernet_init();
  mongoose_init();

  Serial.println("=== Phase 1 System Ready ===");
  Serial.println("Phase 1 Classes: ConfigManager, HardwareManager, DiagnosticManager");
  Serial.println("Type 'help' for debug commands");
}

void loop()
{
  // Update diagnostic loop stats
  diagnosticManager.updateLoopStats();

  // Keep original working loop but add Phase 1 features
  mongoose_poll();

  unsigned long now = millis();

  // Simple heartbeat
  if (now - heartbeatTimer >= 1000)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    heartbeatTimer = now;
  }

  // Update diagnostics
  diagnosticManager.update();

  // Periodic diagnostics - disabled by default (uncomment if needed)
  // if (now - diagnosticTimer >= 300000) { // Every 5 minutes
  //   diagnosticManager.printSystemDiagnostics();
  //   diagnosticTimer = now;
  // }

  // Handle serial debug commands through DiagnosticManager
  if (Serial.available())
  {
    handleDebugCommands();
  }
}

void initializePhase1()
{
  Serial.println("Phase 1: Initializing Core Infrastructure...");

  // Initialize ConfigManager
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

  // Initialize HardwareManager
  if (!hardwareManager.begin())
  {
    Serial.println("ERROR: HardwareManager initialization failed!");
    phase1_initialized = false;
    return;
  }

  // Initialize DiagnosticManager
  if (!diagnosticManager.begin())
  {
    Serial.println("ERROR: DiagnosticManager initialization failed!");
    phase1_initialized = false;
    return;
  }

  phase1_initialized = true;
  Serial.println("Phase 1: All core infrastructure classes ready");
  Serial.println("- ConfigManager: Active");
  Serial.println("- HardwareManager: Active");
  Serial.println("- DiagnosticManager: Active");
}

void handleDebugCommands()
{
  String command = Serial.readStringUntil('\n');
  command.trim();

  // Check for commands that other classes should handle
  if (command.equalsIgnoreCase("config"))
  {
    configManager.printConfiguration();
  }
  else if (command.equalsIgnoreCase("hardware"))
  {
    hardwareManager.printHardwareStatus();
  }
  else if (command.equalsIgnoreCase("hwtest"))
  {
    hardwareManager.runHardwareTest();
  }
  else if (command.equalsIgnoreCase("sensors"))
  {
    hardwareManager.testSensors();
  }
  else if (command.equalsIgnoreCase("scan"))
  {
    hardwareManager.scanI2C();
  }
  else if (command.equalsIgnoreCase("defaults"))
  {
    Serial.println("Resetting configuration to defaults...");
    configManager.resetToDefaults();
    Serial.println("Configuration reset complete. System will use new settings on next boot.");
  }
  else if (command.equalsIgnoreCase("save"))
  {
    configManager.saveAll();
    Serial.println("Configuration saved to EEPROM");
  }
  else
  {
    // Route all other commands to DiagnosticManager
    diagnosticManager.processCommand(command);
  }
}