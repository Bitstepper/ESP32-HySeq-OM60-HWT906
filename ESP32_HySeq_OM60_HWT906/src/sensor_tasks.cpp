// sensor_tasks.cpp - Implementazione stub per compatibilità futura con FreeRTOS
#include "sensor_tasks.h"

#ifdef USE_FREERTOS

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "om60_handler.h"
#include "hwt906_handler.h"

// === VARIABILI GLOBALI FREERTOS ===
static TaskHandle_t sensorTaskHandle = NULL;
static QueueHandle_t sensorDataQueue = NULL;
static SemaphoreHandle_t dataMutex = NULL;

static SensorData latestData = {0};
static TaskStats taskStats = {0};

// === TASK PRINCIPALE SENSORI ===
void sensorTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
    
    Serial.println("📡 FreeRTOS Sensor Task started");
    
    while (true) {
        // Timestamp
        uint32_t timestamp = millis();
        
        // Leggi sensori
        SensorData newData;
        newData.timestamp = timestamp;
        newData.valid = false;
        
        // OM60
        if (isOM60Ready()) {
            newData.distance_mm = getOM60Distance();
            newData.filtered_distance_mm = getFilteredOM60Distance();
            newData.valid = true;
        } else {
            newData.distance_mm = 0;
            newData.filtered_distance_mm = 0;
        }
        
        // HWT906
        if (isHWT906Ready()) {
            newData.pitch = getHWT906Pitch();
            newData.yaw = getHWT906Yaw();
            newData.roll = getHWT906Roll();
            newData.valid = true;
        } else {
            newData.pitch = 0;
            newData.yaw = 0;
            newData.roll = 0;
        }
        
        // Aggiorna dati con mutex
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            latestData = newData;
            taskStats.samples_acquired++;
            xSemaphoreGive(dataMutex);
        }
        
        // Invia a queue se necessario
        if (sensorDataQueue != NULL) {
            xQueueSend(sensorDataQueue, &newData, 0);
        }
        
        // Attendi prossimo ciclo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// === INIZIALIZZAZIONE ===
bool initSensorTasks() {
    Serial.println("🚀 Inizializzazione FreeRTOS sensor tasks...");
    
    // Crea mutex
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("❌ Errore creazione mutex");
        return false;
    }
    
    // Crea queue (opzionale)
    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL) {
        Serial.println("⚠️ Queue non creata (opzionale)");
    }
    
    // Crea task
    BaseType_t result = xTaskCreatePinnedToCore(
        sensorTask,           // Funzione
        "SensorTask",         // Nome
        4096,                 // Stack size
        NULL,                 // Parametri
        2,                    // Priorità (2 = sopra normale)
        &sensorTaskHandle,    // Handle
        1                     // Core 1 (lascia Core 0 per WiFi)
    );
    
    if (result != pdPASS) {
        Serial.println("❌ Errore creazione task");
        return false;
    }
    
    Serial.println("✅ FreeRTOS sensor tasks avviati");
    return true;
}

// === LETTURA DATI ===
bool getLatestSensorData(SensorData& data) {
    if (dataMutex == NULL) return false;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = latestData;
        xSemaphoreGive(dataMutex);
        return data.valid;
    }
    
    return false;
}

// === STATISTICHE ===
void getSensorTaskStats(TaskStats& stats) {
    if (dataMutex == NULL) return;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        stats = taskStats;
        
        // Calcola CPU usage se disponibile
        if (sensorTaskHandle != NULL) {
            // TaskStatus_t taskStatus;
            // vTaskGetInfo(sensorTaskHandle, &taskStatus, pdFALSE, eInvalid);
            // stats.cpu_usage = ... // Calcolo CPU
        }
        
        xSemaphoreGive(dataMutex);
    }
}

// === STOP TASKS ===
void stopSensorTasks() {
    Serial.println("🛑 Arresto FreeRTOS sensor tasks...");
    
    // Elimina task
    if (sensorTaskHandle != NULL) {
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = NULL;
    }
    
    // Elimina queue
    if (sensorDataQueue != NULL) {
        vQueueDelete(sensorDataQueue);
        sensorDataQueue = NULL;
    }
    
    // Elimina mutex
    if (dataMutex != NULL) {
        vSemaphoreDelete(dataMutex);
        dataMutex = NULL;
    }
    
    Serial.println("✅ Tasks arrestati");
}

#endif // USE_FREERTOS

// === VERSIONE STUB SENZA FREERTOS ===
// Se FreeRTOS non è abilitato, queste funzioni non fanno nulla
// Questo permette al codice di compilare senza modifiche

#ifndef USE_FREERTOS

// Definizioni minime per compilazione
struct SensorData {
    float distance_mm;
    float filtered_distance_mm;
    float pitch;
    float yaw;
    float roll;
    uint32_t timestamp;
    bool valid;
};

struct TaskStats {
    uint32_t samples_acquired;
    uint32_t errors;
    float cpu_usage;
};

bool initSensorTasks() {
    // Non fa nulla senza FreeRTOS
    return true;
}

bool getLatestSensorData(SensorData& data) {
    // Non usato senza FreeRTOS
    return false;
}

void getSensorTaskStats(TaskStats& stats) {
    // Non usato senza FreeRTOS
}

void stopSensorTasks() {
    // Non fa nulla senza FreeRTOS
}

#endif // !USE_FREERTOS