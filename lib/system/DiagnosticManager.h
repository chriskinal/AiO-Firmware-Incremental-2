// DiagnosticManager.h
// Phase 1: Debug and Diagnostics Management

#ifndef DIAGNOSTIC_MANAGER_H
#define DIAGNOSTIC_MANAGER_H

#include <Arduino.h>

struct SystemStats
{
    unsigned long uptime_seconds;
    uint32_t free_memory;
    float cpu_usage;
    unsigned long loop_count;
    unsigned long last_loop_time;
    bool phase1_active;
    bool mongoose_active;
};

struct CommandStats
{
    unsigned long total_commands;
    unsigned long unknown_commands;
    unsigned long last_command_time;
    String last_command;
};

class DiagnosticManager
{
private:
    SystemStats system_stats;
    CommandStats command_stats;
    unsigned long last_update_time;
    unsigned long loop_start_time;
    bool initialized;

    // Memory calculation helpers
    uint32_t calculateFreeMemory();
    float calculateCPUUsage();

    // Command processors
    void processStatusCommand();
    void processTestCommand();
    void processPhase1Command();
    void processHelpCommand();
    void processResetCommand();
    void printHelpMenu();

public:
    DiagnosticManager();

    // Lifecycle
    bool begin();
    void update();

    // Command processing
    void processCommand(const String &command);

    // Statistics
    void updateLoopStats();
    SystemStats getSystemStats() const;
    CommandStats getCommandStats() const;

    // Diagnostics output
    void printSystemDiagnostics();
    void printDetailedStats();

    // Status
    bool isInitialized() const { return initialized; }
};

// Memory helper functions for Teensy 4.1
extern "C"
{
    extern char _ebss;   // End of BSS section
    extern char _estack; // End of stack
}

#endif // DIAGNOSTIC_MANAGER_H