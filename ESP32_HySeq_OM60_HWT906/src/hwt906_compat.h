// hwt906_compat.h
// Header di compatibilità per mappare le funzioni chiamate nel main
// alle funzioni della nuova implementazione
// VERSIONE CORRETTA - Allineata agli artifacts hwt906_handler

#ifndef HWT906_COMPAT_H
#define HWT906_COMPAT_H

#include "hwt906_handler.h"
#include <Preferences.h>

// === DEFINIZIONI DI COMPATIBILITÀ ===

// Modalità (mapping ai nuovi enum) - CORRETTO per allinearsi agli artifacts
#define PRECISION_STATIC_LOW      HWT906_MODE_STATIC_LOW
#define PRECISION_STATIC_MEDIUM   HWT906_MODE_STATIC_MED  
#define PRECISION_STATIC_HIGH     HWT906_MODE_STATIC_HIGH
#define PRECISION_DYNAMIC         HWT906_MODE_DYNAMIC

// Alias per compatibilità
typedef HWT906PrecisionMode PrecisionMode;

// === FUNZIONI DI COMPATIBILITÀ ===

// Configurazione modalità 
inline void configureHWT906Mode(PrecisionMode mode) {
    hwt906.setPrecisionMode(mode);  // Usa l'istanza globale
}

// Compensazione drift
inline void enableHWT906DriftCompensation(bool enable) {
    hwt906.setDriftCompensation(enable);
}

// Update (parsing dati)
inline void updateHWT906() {
    hwt906.update();  // Usa il metodo completo che include parsing
}

// Update rate
inline float getHWT906UpdateRate() {
    return hwt906.getUpdateRate();
}

// Zero angoli - VERSIONE MIGLIORATA
inline void zeroHWT906Angles() {
    // Usa il metodo integrato della classe
    hwt906.saveCurrentAsZero();
    Serial.println("✅ HWT906: Angoli azzerati");
}



// Debug - VERSIONE COMPLETA E CORRETTA
inline void printHWT906Debug() {
    const HWT906Data& data = hwt906.getData();  // Accesso sicuro ai dati
    
    Serial.println("\n=== HWT906 DEBUG ===");
    Serial.printf("Angoli: Pitch=%.2f° Yaw=%.2f°\n", 
                  data.pitch_filtered, data.yaw_filtered);
    Serial.printf("Raw: Pitch=%.2f° Yaw=%.2f°\n", 
                  data.pitch_raw, data.yaw_raw);
    Serial.printf("Giroscopio: X=%.2f Y=%.2f Z=%.2f °/s\n", 
                  data.gx, data.gy, data.gz);
    Serial.printf("Accelerometro: X=%.2f Y=%.2f Z=%.2f g\n", 
                  data.ax, data.ay, data.az);
    Serial.printf("Magnetometro: X=%.1f Y=%.1f Z=%.1f\n", 
                  data.mx, data.my, data.mz);
    Serial.printf("Update Rate: %.1f Hz\n", data.update_rate);
    Serial.printf("Frame Count: %lu Error: %lu (%.1f%%)\n", 
                  data.frame_count, 
                  data.error_frames,
                  data.frame_count > 0 ? 100.0f * data.error_frames / data.frame_count : 0.0f);
    
    // Stato sistema
    Serial.println("\n--- Stato Sistema ---");
    Serial.printf("Stato: %s\n", data.is_static ? "STATICO" : "DINAMICO");
    Serial.printf("Calibrato: %s\n", data.is_calibrated ? "SÌ" : "NO");
    Serial.printf("Durata statico: %lu ms\n", 
                  data.is_static ? (millis() - data.static_start_time) : 0);
    
    // Range info
    Serial.println("\n--- Range Registrati ---");
    Serial.printf("Pitch: %.1f° ÷ %.1f°\n", data.pitch_min, data.pitch_max);
    Serial.printf("Yaw: %.1f° ÷ %.1f°\n", data.yaw_min, data.yaw_max);
    
    // Offset calibrazione
    if (data.is_calibrated) {
        Serial.println("\n--- Calibrazione ---");
        Serial.printf("Offset Pitch: %.2f°\n", data.pitch_offset);
        Serial.printf("Offset Yaw: %.2f°\n", data.yaw_offset);
        Serial.printf("Offset Roll: %.2f°\n", data.roll_offset);
    }
    
    Serial.println("==================\n");
}

// Check if ready (presente e aggiornato di recente)
inline bool isHWT906Ready() {
    return hwt906.isReady();
}

// Funzioni aggiuntive per completezza API
inline bool isHWT906Present() {
    return hwt906.isReady();
}

inline void calibrateHWT906() {
    Serial.println("🎯 Avvio calibrazione HWT906...");
    hwt906.startCalibration();
}

// Getters diretti (per compatibilità)
inline float getHWT906Pitch() {
    return hwt906.getPitch();
}

inline float getHWT906Yaw() {
    return hwt906.getYaw(); 
}

inline float getHWT906Roll() {
    return hwt906.getRoll();
}

// Stato calibrazione
inline bool isHWT906Calibrated() {
    return hwt906.isCalibrated();
}

// Reset calibrazione
inline void resetHWT906Calibration() {
    hwt906.resetCalibration();
    Serial.println("🔄 Calibrazione HWT906 resettata");
}

// Configurazione zone morte
inline void setHWT906DeadZones(float pitch_zone, float yaw_zone) {
    hwt906.setDeadZones(pitch_zone, yaw_zone);
}

// Info range (per diagnostica)
inline void getHWT906Range(float& pitch_min, float& pitch_max, float& yaw_min, float& yaw_max) {
    hwt906.getRange(pitch_min, pitch_max, yaw_min, yaw_max);
}

// Reset range
inline void resetHWT906Range() {
    hwt906.resetRange();
}

// === MACRO DI COMPATIBILITÀ AGGIUNTIVE ===
#define HWT906_BAUD_RATE 921600  // Alta velocità come negli artifacts
#define HWT906_TIMEOUT   1000

// === COMPATIBILITY ALIASES PER CODICE LEGACY ===
#define initIMU() initHWT906()
#define getIMUPitch() getHWT906Pitch()
#define getIMUYaw() getHWT906Yaw()
#define isIMUReady() isHWT906Ready()

#endif // HWT906_COMPAT_H
