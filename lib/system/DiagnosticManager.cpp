// DiagnosticManager.cpp
// Phase 1: Debug and Diagnostics Management

#include "DiagnosticManager.h"

DiagnosticManager::DiagnosticManager() : initialized(false), last_update_time(0), loop_start_time(0)
{
    // Initialize stats structures
    memset(&system_stats, 0, sizeof(SystemStats));
    memset(&command_stats, 0, sizeof(CommandStats));
}

bool DiagnosticManager::begin()
{
    Serial.println("DiagnosticManager: Initializing...");

    // Initialize timing
    last_update_time = millis();
    loop_start_time = micros();

    // Initialize stats
    system_stats.phase1_active = true;
    system_stats.mongoose_active = true;
    command_stats.total_commands = 0;
    command_stats.unknown_commands = 0;

    initialized = true;
    Serial.println("DiagnosticManager: Ready");
    return true;
}

void DiagnosticManager::update()
{
    if (!initialized)
        return;

    unsigned long now = millis();
    if (now - last_update_time >= 1000)
    { // Update every second
        // Update system stats
        system_stats.uptime_seconds = now / 1000;
        system_stats.free_memory = calculateFreeMemory();
        system_stats.cpu_usage = calculateCPUUsage();

        last_update_time = now;
    }
}

void DiagnosticManager::updateLoopStats()
{
    if (!initialized)
        return;

    unsigned long now = micros();
    system_stats.last_loop_time = now - loop_start_time;
    system_stats.loop_count++;
    loop_start_time = now;
}

void DiagnosticManager::processCommand(const String &command)
{
    if (!initialized)
        return;

    String cmd = command;
    cmd.trim();
    cmd.toLowerCase();

    // Update command stats
    command_stats.total_commands++;
    command_stats.last_command = cmd;
    command_stats.last_command_time = millis();

    // Process commands
    if (cmd == "status")
    {
        processStatusCommand();
    }
    else if (cmd == "test")
    {
        processTestCommand();
    }
    else if (cmd == "phase1")
    {
        processPhase1Command();
    }
    else if (cmd == "stats")
    {
        printDetailedStats();
    }
    else if (cmd == "help")
    {
        processHelpCommand();
    }
    else if (cmd == "reset")
    {
        processResetCommand();
    }
    else if (cmd.length() > 0)
    {
        command_stats.unknown_commands++;
        Serial.printf("Unknown command: %s\n", cmd.c_str());
        Serial.println("Type 'help' for available commands");
    }
}

void DiagnosticManager::processStatusCommand()
{
    printSystemDiagnostics();
}

void DiagnosticManager::processTestCommand()
{
    Serial.println("Running Phase 1 diagnostic test...");

    // Test LED
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    Serial.println("Diagnostic test complete");
}

void DiagnosticManager::processPhase1Command()
{
    Serial.printf("Phase 1 Status: %s\n", system_stats.phase1_active ? "Active" : "Inactive");
    Serial.println("Phase 1 Components:");
    Serial.println("- ConfigManager: Active");
    Serial.println("- HardwareManager: Active");
    Serial.println("- DiagnosticManager: Active");
    Serial.println("Ready for Phase 2: NetworkManager, SerialManager, CANManager");
}

void DiagnosticManager::processHelpCommand()
{
    printHelpMenu();
}

void DiagnosticManager::processResetCommand()
{
    Serial.println("Performing system reset...");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; // Teensy 4.x software reset
}

void DiagnosticManager::printSystemDiagnostics()
{
    Serial.println("=== System Diagnostics ===");
    Serial.printf("Uptime: %lu seconds\n", system_stats.uptime_seconds);
    Serial.printf("Free RAM: %lu bytes\n", system_stats.free_memory);
    Serial.printf("CPU Usage: %.1f%%\n", system_stats.cpu_usage);
    Serial.printf("Loop Count: %lu\n", system_stats.loop_count);
    Serial.printf("Last Loop: %lu μs\n", system_stats.last_loop_time);
    Serial.printf("Phase 1: %s\n", system_stats.phase1_active ? "Active" : "Inactive");
    Serial.printf("Mongoose: %s\n", system_stats.mongoose_active ? "Active" : "Inactive");
    Serial.println("========================");
}

void DiagnosticManager::printDetailedStats()
{
    Serial.println("=== Detailed Statistics ===");

    // System stats
    Serial.println("System:");
    Serial.printf("  Uptime: %lu seconds\n", system_stats.uptime_seconds);
    Serial.printf("  Free Memory: %lu bytes\n", system_stats.free_memory);
    Serial.printf("  CPU Usage: %.1f%%\n", system_stats.cpu_usage);
    Serial.printf("  Loop Count: %lu\n", system_stats.loop_count);
    Serial.printf("  Last Loop Time: %lu μs\n", system_stats.last_loop_time);

    // Command stats
    Serial.println("Commands:");
    Serial.printf("  Total Commands: %lu\n", command_stats.total_commands);
    Serial.printf("  Unknown Commands: %lu\n", command_stats.unknown_commands);
    Serial.printf("  Last Command: %s\n", command_stats.last_command.c_str());
    Serial.printf("  Last Command Time: %lu\n", command_stats.last_command_time);

    // Component status
    Serial.println("Components:");
    Serial.printf("  Phase 1 Active: %s\n", system_stats.phase1_active ? "Yes" : "No");
    Serial.printf("  Mongoose Active: %s\n", system_stats.mongoose_active ? "Yes" : "No");

    Serial.println("===========================");
}

void DiagnosticManager::printHelpMenu()
{
    Serial.println("=== Phase 1 Debug Commands ===");
    Serial.println("status   - Show system status");
    Serial.println("stats    - Show detailed statistics");
    Serial.println("test     - Run diagnostic test");
    Serial.println("phase1   - Show Phase 1 component status");
    Serial.println("help     - Show this menu");
    Serial.println("reset    - Reset system");
    Serial.println("==============================");
}

uint32_t DiagnosticManager::calculateFreeMemory()
{
    char stack_top;
    return &stack_top - &_ebss;
}

float DiagnosticManager::calculateCPUUsage()
{
    // Simple CPU usage estimation based on loop timing
    if (system_stats.last_loop_time > 0)
    {
        float usage = (system_stats.last_loop_time / 1000.0) * 100.0; // Convert μs to %
        return min(usage, 100.0f);
    }
    return 0.0f;
}

SystemStats DiagnosticManager::getSystemStats() const
{
    return system_stats;
}

CommandStats DiagnosticManager::getCommandStats() const
{
    return command_stats;
}