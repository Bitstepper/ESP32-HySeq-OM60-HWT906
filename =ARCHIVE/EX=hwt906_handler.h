// hwt906_handler_improved.h
// Header migliorato con supporto filtri avanzati e compensazione drift

#ifndef HWT906_HANDLER_H
#define HWT906_HANDLER_H

#include <Arduino.h>

// === CONFIGURAZIONE HARDWARE ===
#define HWT906_I2C_ADDR 0x50     // Indirizzo I2C per configurazione
#define HWT906_RX_PIN   44       // UART RX per stream dati
#define HWT906_TX_PIN   43       // UART TX per comandi

// === MODALITÀ PRECISIONE ===
enum PrecisionMode {
    PRECISION_STATIC_LOW = 0,     // 5Hz, filtri massimi
    PRECISION_STATIC_MEDIUM = 1,  // 10Hz, filtri alti (default)
    PRECISION_STATIC_HIGH = 2,    // 20Hz, filtri medi
    PRECISION_DYNAMIC = 3         // 50Hz, per confronto
};

// === STRUTTURA DATI PRINCIPALE ===
struct HWT906Data {
    // Angoli filtrati (output principale)
    float pitch;                  // -90° to +90°
    float yaw;                    // -180° to +180°
    
    // Angoli raw (prima dei filtri)
    float pitch_raw;
    float yaw_raw;
    
    // Dati grezzi sensori
    float ax, ay, az;            // Accelerometro (g)
    float gx, gy, gz;            // Giroscopio (°/s)
    float mx, my, mz;            // Magnetometro (non usato)
    
    // Temperatura
    float temperature;           // °C
    
    // Calibrazione
    float pitch_offset;
    float yaw_offset;
    
    // Statistiche comunicazione
    uint32_t frames_received;
    uint32_t frames_error;
    uint32_t last_frame_count;
    float update_rate;           // Hz
    
    // Stato sensore
    bool sensor_present;
    bool valid;
    uint32_t last_update;
};

// === FUNZIONI PRINCIPALI ===

// Inizializzazione
bool initHWT906();

// Lettura dati (chiamare nel loop)
void parseHWT906Data();

// Accesso valori filtrati
float getHWT906Pitch();
float getHWT906Yaw();

// Stato sensore
bool isHWT906Present();

// Calibrazione (20 secondi)
void calibrateHWT906();

// === FUNZIONI AVANZATE ===

// Funzioni calibrazione magnetometro (stub per compatibilità)
void startMagCalibration();
bool isMagCalibrationInProgress();
float getMagCalibrationProgress();
void finishMagCalibration();

// Modalità precisione
void setPrecisionMode(PrecisionMode mode);
PrecisionMode getPrecisionMode();

// Compensazione drift
void setDriftCompensation(bool enabled);
bool isDriftCompensationEnabled();
float getDriftRate();  // °/s

// Metriche stabilità
void getStabilityMetrics(float& pitch_sigma, float& yaw_sigma);
bool isStaticState();
float getGyroMagnitude();

// Debug e diagnostica
void printHWT906Status();
void printFilterStates();

// === COSTANTI UTILI ===
#define HWT906_UPDATE_TIMEOUT 1000   // ms - timeout per presenza sensore
#define HWT906_CALIB_DURATION 20000  // ms - durata calibrazione
#define HWT906_MIN_SAMPLES    30     // Campioni minimi per statistiche

// === COSTANTI FILTRI ===
#define MOVING_AVG_SIZE 10
#define YAW_DRIFT_WINDOW 300

// === STRUTTURE FILTRI AVANZATI ===
struct AdvancedFilters {
    // Moving average buffers
    float pitch_buffer[MOVING_AVG_SIZE];
    float yaw_buffer[MOVING_AVG_SIZE];
    uint8_t buffer_index;
    bool buffer_filled;
    
    // Complementary filter states
    float comp_pitch;
    float comp_yaw;
    
    // Kalman filter per YAW
    float kalman_yaw;
    float kalman_P;
    float kalman_Q;
    float kalman_R;
    
    // Static detection
    uint32_t static_start_time;
    bool is_static;
    float last_gyro_magnitude;
};

struct DriftCompensation {
    float yaw_history[YAW_DRIFT_WINDOW];
    uint32_t timestamps[YAW_DRIFT_WINDOW];
    uint16_t history_index;
    bool history_filled;
    
    float yaw_drift_rate;
    float yaw_reference;
    float yaw_accumulated;
    uint32_t last_compensation_time;
    
    bool enabled;
};

// === VARIABILI GLOBALI ESTERNE ===
extern HWT906Data currentData;
extern AdvancedFilters filters;
extern DriftCompensation drift_comp;

#endif // HWT906_HANDLER_H
