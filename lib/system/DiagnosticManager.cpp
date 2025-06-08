// DiagnosticManager.cpp
// Phase 1: Diagnostics and Statistics Implementation

#include "DiagnosticManager.h"

// External memory function
extern "C" char *sbrk(int incr);

DiagnosticManager::DiagnosticManager()
{
    task_count = 0;
    for (uint8_t i = 0; i < MAX_CPU_TASKS; i++)
    {
        cpu_tasks[i] = nullptr;
    }

    // Initialize structures
    memset(&system_stats, 0, sizeof(system_stats));
    memset(&perf_counters, 0, sizeof(perf_counters));

    // Set defaults
    verbose_output = false;
    print_cpu_usage = false;
    print_performance = false;
    last_loop_time = 0;
    loop_time_sum = 0;
    loop_count_for_avg = 0;
    min_free_memory = 0xFFFFFFFF;
    startup_memory = 0;
}

bool DiagnosticManager::begin()
{
    Serial.println("DiagnosticManager: Initializing diagnostics...");

    // Record startup memory
    startup_memory = getFreeMemory();
    min_free_memory = startup_memory;

    // Initialize system stats
    system_stats.uptime_seconds = 0;
    system_stats.free_memory = startup_memory;
    system_stats.loop_count = 0;
    system_stats.loop_time_avg = 0;
    system_stats.loop_time_max = 0;
    system_stats.cpu_usage_percent = 0.0;

    // Reset all counters
    resetCounters();

    Serial.printf("DiagnosticManager: Initialized with %lu bytes free memory\n", startup_memory);
    return true;
}

CPUUsage *DiagnosticManager::createTask(const char *name)
{
    if (task_count >= MAX_CPU_TASKS)
    {
        Serial.printf("DiagnosticManager: Cannot create task '%s' - limit reached\n", name);
        return nullptr;
    }

    cpu_tasks[task_count] = new CPUUsage(name);
    if (cpu_tasks[task_count] == nullptr)
    {
        Serial.printf("DiagnosticManager: Failed to allocate memory for task '%s'\n", name);
        return nullptr;
    }

    task_count++;
    Serial.printf("DiagnosticManager: Created CPU task '%s'\n", name);
    return cpu_tasks[task_count - 1];
}

void DiagnosticManager::startTask(CPUUsage *task)
{
    if (task && !task->active)
    {
        task->start_time = micros();
        task->active = true;
    }
}

void DiagnosticManager::endTask(CPUUsage *task)
{
    if (task && task->active)
    {
        uint32_t elapsed = micros() - task->start_time;
        task->total_time += elapsed;
        task->call_count++;
        if (elapsed > task->max_time)
        {
            task->max_time = elapsed;
        }
        task->active = false;
    }
}

void DiagnosticManager::printCPUUsage()
{
    Serial.println("=== CPU Usage Statistics ===");
    Serial.printf("%-12s %8s %8s %8s %8s\n", "Task", "Calls", "Total", "Max", "Avg");
    Serial.println("--------------------------------------------");

    for (uint8_t i = 0; i < task_count; i++)
    {
        CPUUsage *task = cpu_tasks[i];
        if (task && task->call_count > 0)
        {
            uint32_t avg_time = task->total_time / task->call_count;
            Serial.printf("%-12s %8lu %8lu %8lu %8lu\n",
                          task->name,
                          task->call_count,
                          task->total_time,
                          task->max_time,
                          avg_time);
        }
    }
    Serial.println("=============================");
}

void DiagnosticManager::incrementCounter(const char *counter_name)
{
    if (strcmp(counter_name, "mongoose_polls") == 0)
    {
        perf_counters.mongoose_polls++;
    }
    else if (strcmp(counter_name, "gps_messages") == 0)
    {
        perf_counters.gps_messages++;
    }
    else if (strcmp(counter_name, "udp_sent") == 0)
    {
        perf_counters.udp_packets_sent++;
    }
    else if (strcmp(counter_name, "udp_received") == 0)
    {
        perf_counters.udp_packets_received++;
    }
    else if (strcmp(counter_name, "can_messages") == 0)
    {
        perf_counters.can_messages++;
    }
    else if (strcmp(counter_name, "i2c_transactions") == 0)
    {
        perf_counters.i2c_transactions++;
    }
    else if (strcmp(counter_name, "config_saves") == 0)
    {
        perf_counters.config_saves++;
    }
    else if (strcmp(counter_name, "hardware_errors") == 0)
    {
        perf_counters.hardware_errors++;
    }
}

void DiagnosticManager::setCounter(const char *counter_name, uint32_t value)
{
    if (strcmp(counter_name, "mongoose_polls") == 0)
    {
        perf_counters.mongoose_polls = value;
    }
    else if (strcmp(counter_name, "gps_messages") == 0)
    {
        perf_counters.gps_messages = value;
    }
    // Add other counters as needed
}

uint32_t DiagnosticManager::getCounter(const char *counter_name)
{
    if (strcmp(counter_name, "mongoose_polls") == 0)
    {
        return perf_counters.mongoose_polls;
    }
    else if (strcmp(counter_name, "gps_messages") == 0)
    {
        return perf_counters.gps_messages;
    }
    else if (strcmp(counter_name, "udp_sent") == 0)
    {
        return perf_counters.udp_packets_sent;
    }
    else if (strcmp(counter_name, "udp_received") == 0)
    {
        return perf_counters.udp_packets_received;
    }
    return 0;
}

void DiagnosticManager::printPerformanceCounters()
{
    Serial.println("=== Performance Counters ===");
    Serial.printf("Mongoose Polls: %lu\n", perf_counters.mongoose_polls);
    Serial.printf("GPS Messages: %lu\n", perf_counters.gps_messages);
    Serial.printf("UDP Sent: %lu\n", perf_counters.udp_packets_sent);
    Serial.printf("UDP Received: %lu\n", perf_counters.udp_packets_received);
    Serial.printf("CAN Messages: %lu\n", perf_counters.can_messages);
    Serial.printf("I2C Transactions: %lu\n", perf_counters.i2c_transactions);
    Serial.printf("Config Saves: %lu\n", perf_counters.config_saves);
    Serial.printf("Hardware Errors: %lu\n", perf_counters.hardware_errors);
    Serial.println("============================");
}

uint32_t DiagnosticManager::getFreeMemory()
{
    char top;
    return &top - reinterpret_cast<char *>(sbrk(0));
}

void DiagnosticManager::updateSystemStats()
{
    system_stats.uptime_seconds = millis() / 1000;
    system_stats.free_memory = getFreeMemory();

    // Track minimum free memory
    if (system_stats.free_memory < min_free_memory)
    {
        min_free_memory = system_stats.free_memory;
    }

    // Calculate CPU usage percentage (rough estimate)
    if (stats_update_timer >= 1000)
    {
        uint32_t total_cpu_time = 0;
        for (uint8_t i = 0; i < task_count; i++)
        {
            if (cpu_tasks[i])
            {
                total_cpu_time += cpu_tasks[i]->total_time;
                // Reset for next measurement period
                cpu_tasks[i]->total_time = 0;
                cpu_tasks[i]->call_count = 0;
            }
        }

        // Very rough CPU usage estimate
        system_stats.cpu_usage_percent = (total_cpu_time / 10000.0); // Convert to percentage
        if (system_stats.cpu_usage_percent > 100.0)
        {
            system_stats.cpu_usage_percent = 100.0;
        }

        stats_update_timer = 0;
    }
}

void DiagnosticManager::updateStats()
{
    updateSystemStats();
    trackMemoryUsage();
}

void DiagnosticManager::printSystemStats()
{
    updateSystemStats();

    Serial.println("=== System Statistics ===");
    Serial.printf("Uptime: %lu seconds\n", system_stats.uptime_seconds);
    Serial.printf("Free Memory: %lu bytes\n", system_stats.free_memory);
    Serial.printf("Min Free Memory: %lu bytes\n", min_free_memory);
    Serial.printf("Memory Used: %lu bytes\n", startup_memory - system_stats.free_memory);
    Serial.printf("Loop Count: %lu\n", system_stats.loop_count);
    Serial.printf("Avg Loop Time: %lu us\n", system_stats.loop_time_avg);
    Serial.printf("Max Loop Time: %lu us\n", system_stats.loop_time_max);
    Serial.printf("CPU Usage: %.1f%%\n", system_stats.cpu_usage_percent);
    Serial.println("=========================");
}

void DiagnosticManager::trackMemoryUsage()
{
    uint32_t current_free = getFreeMemory();
    if (current_free < min_free_memory)
    {
        min_free_memory = current_free;
        if (verbose_output)
        {
            Serial.printf("DiagnosticManager: New minimum free memory: %lu bytes\n", min_free_memory);
        }
    }
}

void DiagnosticManager::recordLoopTime(uint32_t loop_time)
{
    system_stats.loop_count++;

    // Update maximum
    if (loop_time > system_stats.loop_time_max)
    {
        system_stats.loop_time_max = loop_time;
    }

    // Calculate rolling average
    loop_time_sum += loop_time;
    loop_count_for_avg++;

    if (loop_count_for_avg >= 100)
    { // Update average every 100 loops
        system_stats.loop_time_avg = loop_time_sum / loop_count_for_avg;
        loop_time_sum = 0;
        loop_count_for_avg = 0;
    }
}

void DiagnosticManager::resetCounters()
{
    memset(&perf_counters, 0, sizeof(perf_counters));

    // Reset CPU task counters
    for (uint8_t i = 0; i < task_count; i++)
    {
        if (cpu_tasks[i])
        {
            cpu_tasks[i]->total_time = 0;
            cpu_tasks[i]->max_time = 0;
            cpu_tasks[i]->call_count = 0;
        }
    }

    system_stats.loop_time_max = 0;
    Serial.println("DiagnosticManager: All counters reset");
}

void DiagnosticManager::resetAllCounters()
{
    resetCounters();
    min_free_memory = getFreeMemory();
}

void DiagnosticManager::printFullDiagnostics()
{
    Serial.println("=== FULL SYSTEM DIAGNOSTICS ===");
    printSystemStats();
    Serial.println();
    printPerformanceCounters();
    Serial.println();
    printCPUUsage();
    Serial.println("===============================");
}

bool DiagnosticManager::isSystemHealthy()
{
    updateSystemStats();

    // Check memory usage
    if (system_stats.free_memory < 10000)
    { // Less than 10KB free
        return false;
    }

    // Check loop time
    if (system_stats.loop_time_max > 100000)
    { // Loop taking > 100ms
        return false;
    }

    // Check CPU usage
    if (system_stats.cpu_usage_percent > 95.0)
    {
        return false;
    }

    return true;
}

void DiagnosticManager::checkSystemHealth()
{
    if (!isSystemHealthy())
    {
        Serial.println("DiagnosticManager: WARNING - System health check failed!");
        printSystemStats();
    }
}

String DiagnosticManager::getJSONStats()
{
    updateSystemStats();

    String json = "{";
    json += "\"uptime\":" + String(system_stats.uptime_seconds) + ",";
    json += "\"free_memory\":" + String(system_stats.free_memory) + ",";
    json += "\"loop_avg\":" + String(system_stats.loop_time_avg) + ",";
    json += "\"loop_max\":" + String(system_stats.loop_time_max) + ",";
    json += "\"cpu_usage\":" + String(system_stats.cpu_usage_percent) + ",";
    json += "\"mongoose_polls\":" + String(perf_counters.mongoose_polls) + ",";
    json += "\"gps_messages\":" + String(perf_counters.gps_messages) + ",";
    json += "\"udp_sent\":" + String(perf_counters.udp_packets_sent) + ",";
    json += "\"udp_received\":" + String(perf_counters.udp_packets_received);
    json += "}";

    return json;
}

void DiagnosticManager::processDebugCommand(const String &command)
{
    if (command == "stats")
    {
        printSystemStats();
    }
    else if (command == "cpu")
    {
        printCPUUsage();
    }
    else if (command == "perf")
    {
        printPerformanceCounters();
    }
    else if (command == "full")
    {
        printFullDiagnostics();
    }
    else if (command == "reset")
    {
        resetAllCounters();
    }
    else if (command == "health")
    {
        checkSystemHealth();
    }
    else if (command == "verbose")
    {
        verbose_output = !verbose_output;
        Serial.printf("Verbose output: %s\n", verbose_output ? "ON" : "OFF");
    }
    else if (command == "json")
    {
        Serial.println(getJSONStats());
    }
}