// sensor_tasks.h - Stub per compatibilità futura con FreeRTOS
#ifndef SENSOR_TASKS_H
#define SENSOR_TASKS_H

#include <Arduino.h>

// Disabilita FreeRTOS per questo progetto
// #define USE_FREERTOS

#ifdef USE_FREERTOS

// Struttura dati sensori per FreeRTOS
struct SensorData {
    float distance_mm;
    float filtered_distance_mm;
    float pitch;
    float yaw;
    float roll;
    uint32_t timestamp;
    bool valid;
};

// Struttura statistiche
struct TaskStats {
    uint32_t samples_acquired;
    uint32_t errors;
    float cpu_usage;
};

// API FreeRTOS (da implementare se necessario)
bool initSensorTasks();
bool getLatestSensorData(SensorData& data);
void getSensorTaskStats(TaskStats& stats);
void stopSensorTasks();

#endif // USE_FREERTOS

#endif // SENSOR_TASKS_H