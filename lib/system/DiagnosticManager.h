// DiagnosticManager.h
// Phase 1: Diagnostics and Statistics Collection

#ifndef DIAGNOSTIC_MANAGER_H
#define DIAGNOSTIC_MANAGER_H

#include "Arduino.h"
#include "elapsedMillis.h"

// CPU usage tracking structure
struct CPUUsage
{
    const char *name;
    uint32_t start_time;
    uint32_t total_time;
    uint32_t max_time;
    uint32_t call_count;
    bool active;

    CPUUsage(const char *task_name) : name(task_name), start_time(0), total_time(0),
                                      max_time(0), call_count(0), active(false) {}
};

// System statistics structure
struct SystemStats
{
    uint32_t uptime_seconds;
    uint32_t free_memory;
    uint32_t loop_count;
    uint32_t loop_time_avg;
    uint32_t loop_time_max;
    float cpu_usage_percent;
    uint32_t last_reset_reason;
};

// Performance tracking for different subsystems
struct PerformanceCounters
{
    uint32_t mongoose_polls;
    uint32_t gps_messages;
    uint32_t udp_packets_sent;
    uint32_t udp_packets_received;
    uint32_t can_messages;
    uint32_t i2c_transactions;
    uint32_t config_saves;
    uint32_t hardware_errors;
};

class DiagnosticManager
{
private:
    static const uint8_t MAX_CPU_TASKS = 16;

    CPUUsage *cpu_tasks[MAX_CPU_TASKS];
    uint8_t task_count;

    SystemStats system_stats;
    PerformanceCounters perf_counters;

    // Timing variables
    elapsedMillis diagnostic_timer;
    elapsedMillis stats_update_timer;
    uint32_t last_loop_time;
    uint32_t loop_time_sum;
    uint32_t loop_count_for_avg;

    // Memory tracking
    uint32_t min_free_memory;
    uint32_t startup_memory;

    // Debug output control
    bool verbose_output;
    bool print_cpu_usage;
    bool print_performance;

    // Helper methods
    uint32_t getFreeMemory();
    void updateSystemStats();
    void resetCounters();

public:
    DiagnosticManager();

    // Initialize diagnostics
    bool begin();

    // CPU usage tracking
    CPUUsage *createTask(const char *name);
    void startTask(CPUUsage *task);
    void endTask(CPUUsage *task);
    void printCPUUsage();

    // Performance counters
    void incrementCounter(const char *counter_name);
    void setCounter(const char *counter_name, uint32_t value);
    uint32_t getCounter(const char *counter_name);
    void printPerformanceCounters();

    // System statistics
    void updateStats();
    const SystemStats &getSystemStats() const { return system_stats; }
    void printSystemStats();

    // Memory diagnostics
    void trackMemoryUsage();
    uint32_t getMinFreeMemory() const { return min_free_memory; }
    uint32_t getMemoryUsage() const { return startup_memory - getFreeMemory(); }

    // Loop timing
    void recordLoopTime(uint32_t loop_time);
    uint32_t getAverageLoopTime() const { return system_stats.loop_time_avg; }
    uint32_t getMaxLoopTime() const { return system_stats.loop_time_max; }

    // Debug output control
    void enableVerboseOutput(bool enable) { verbose_output = enable; }
    void enableCPUUsagePrint(bool enable) { print_cpu_usage = enable; }
    void enablePerformancePrint(bool enable) { print_performance = enable; }

    // Comprehensive diagnostics
    void printFullDiagnostics();
    void resetAllCounters();

    // Health monitoring
    bool isSystemHealthy();
    void checkSystemHealth();

    // Export diagnostics (for web interface)
    String getJSONStats();

    // Debug commands
    void processDebugCommand(const String &command);
};

// Helper macros for easy CPU tracking
#define DIAGNOSTIC_START(manager, task) \
    if (task)                           \
    manager.startTask(task)
#define DIAGNOSTIC_END(manager, task) \
    if (task)                         \
    manager.endTask(task)

#endif // DIAGNOSTIC_MANAGER_H