// hwt906_compat.h
// Header di compatibilità per mappare le funzioni chiamate nel main
// alle funzioni della nuova implementazione

#ifndef HWT906_COMPAT_H
#define HWT906_COMPAT_H

#include "hwt906_handler.h"
#include <Preferences.h>

// === DEFINIZIONI DI COMPATIBILITÀ ===

// Modalità (mapping ai nuovi enum)
#define HWT906_MODE_STATIC_LOW    PRECISION_STATIC_LOW
#define HWT906_MODE_STATIC_MED    PRECISION_STATIC_MEDIUM
#define HWT906_MODE_STATIC_HIGH   PRECISION_STATIC_HIGH
#define HWT906_MODE_DYNAMIC       PRECISION_DYNAMIC

// === FUNZIONI DI COMPATIBILITÀ ===

// Configurazione modalità
inline void configureHWT906Mode(PrecisionMode mode) {
    setPrecisionMode(mode);
}

// Compensazione drift
inline void enableHWT906DriftCompensation(bool enable) {
    setDriftCompensation(enable);
}

// Update (parsing dati)
inline void updateHWT906() {
    parseHWT906Data();
}

// Update rate
inline float getHWT906UpdateRate() {
    extern HWT906Data currentData;  // Accesso alla struttura dati globale
    return currentData.update_rate;
}

// Zero angoli
inline void zeroHWT906Angles() {
    // Salva gli angoli correnti come offset
    extern HWT906Data currentData;
    currentData.pitch_offset = currentData.pitch_raw;
    currentData.yaw_offset = currentData.yaw_raw;
    
    // Salva in preferences
    Preferences prefs;
    prefs.begin("hwt906", false);
    prefs.putFloat("pitch_off", currentData.pitch_offset);
    prefs.putFloat("yaw_off", currentData.yaw_offset);
    prefs.end();
    
    Serial.println("✅ HWT906: Angoli azzerati");
}

// Debug
inline void printHWT906Debug() {
    extern HWT906Data currentData;
    extern AdvancedFilters filters;
    extern DriftCompensation drift_comp;
    
    Serial.println("\n=== HWT906 DEBUG ===");
    Serial.printf("Angoli: Pitch=%.2f° Yaw=%.2f°\n", 
                  currentData.pitch, currentData.yaw);
    Serial.printf("Raw: Pitch=%.2f° Yaw=%.2f°\n", 
                  currentData.pitch_raw, currentData.yaw_raw);
    Serial.printf("Giroscopio: X=%.2f Y=%.2f Z=%.2f °/s\n", 
                  currentData.gx, currentData.gy, currentData.gz);
    Serial.printf("Accelerometro: X=%.2f Y=%.2f Z=%.2f g\n", 
                  currentData.ax, currentData.ay, currentData.az);
    Serial.printf("Temperatura: %.1f°C\n", currentData.temperature);
    Serial.printf("Update Rate: %.1f Hz\n", currentData.update_rate);
    Serial.printf("Frame RX: %lu ERR: %lu (%.1f%%)\n", 
                  currentData.frames_received, 
                  currentData.frames_error,
                  100.0f * currentData.frames_error / 
                  (currentData.frames_received + currentData.frames_error));
    
    // Stato filtri
    Serial.println("\n--- Filtri ---");
    Serial.printf("Stato: %s\n", filters.is_static ? "STATICO" : "DINAMICO");
    Serial.printf("Gyro Magnitude: %.2f °/s\n", filters.last_gyro_magnitude);
    
    // Drift compensation
    if (drift_comp.enabled) {
        Serial.println("\n--- Drift Compensation ---");
        Serial.printf("Drift Rate: %.5f °/s\n", drift_comp.yaw_drift_rate);
        Serial.printf("Accumulated: %.2f°\n", drift_comp.yaw_accumulated);
        Serial.printf("Samples: %d\n", 
                      drift_comp.history_filled ? YAW_DRIFT_WINDOW : drift_comp.history_index);
    }
    
    // Stabilità
    float pitch_sigma, yaw_sigma;
    getStabilityMetrics(pitch_sigma, yaw_sigma);
    Serial.println("\n--- Stabilità ---");
    Serial.printf("σ Pitch: %.3f°\n", pitch_sigma);
    Serial.printf("σ Yaw: %.3f°\n", yaw_sigma);
    
    Serial.println("==================\n");
}

// Check if ready (presente e aggiornato di recente)
inline bool isHWT906Ready() {
    return isHWT906Present();
}

// === MACRO DI COMPATIBILITÀ AGGIUNTIVE ===
// Per eventuali altre funzioni non ancora mappate
#define HWT906_BAUD_RATE 9600
#define HWT906_TIMEOUT   1000


#endif // HWT906_COMPAT_H
