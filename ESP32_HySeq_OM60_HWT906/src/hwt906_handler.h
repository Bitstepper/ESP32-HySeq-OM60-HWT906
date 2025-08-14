// hwt906_handler.h - Versione unificata basata su WIT9-proto funzionante
// Migrazione completa dei filtri e parsing stabili

#ifndef HWT906_HANDLER_H
#define HWT906_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// === CONFIGURAZIONI OTTIMIZZATE DAL WIT9-PROTO ===
#define HWT906_I2C_ADDR 0x50
#define HWT906_UART_SPEED 9600            // CORRETTO: Default HWT906 9600 baud
#define HWT906_EEPROM_BASE 200        // Offset separato da radar

// UART Frame Protocol
#define FRAME_HEADER 0x55
#define FRAME_ACCEL 0x51
#define FRAME_GYRO 0x52  
#define FRAME_ANGLE 0x53
#define FRAME_MAG 0x54
#define FRAME_SIZE 11

// === PRECISIONE OTTIMIZZATA (dal WIT9-proto) ===
enum HWT906PrecisionMode {
    HWT906_MODE_STATIC_LOW = 0,    // 5Hz, massimo filtering
    HWT906_MODE_STATIC_MED = 1,    // 10Hz, bilanciato (DEFAULT)
    HWT906_MODE_STATIC_HIGH = 2,   // 20Hz, più responsive
    HWT906_MODE_DYNAMIC = 3        // 50Hz, solo per test
};

struct HWT906PrecisionConfig {
    uint16_t output_rate;      
    uint16_t bandwidth;        
    uint16_t kalman_k;         
    uint16_t acc_filter;
    uint16_t gyro_threshold;   
    uint16_t gyro_cal_time;    
    const char* description;
};

// === FILTRI AVANZATI MULTI-STADIO (dal WIT9-proto) ===
struct AdvancedFilters {
    // Filtro complementare ottimizzato
    float comp_alpha = 0.98f;  // Per uso statico
    
    // Buffer media mobile (10 campioni)
    float roll_buffer[10];
    float pitch_buffer[10];
    float yaw_buffer[10];
    int buffer_index = 0;
    bool buffer_full = false;
    
    // Kalman semplificato per YAW (componente critica)
    float Q_angle = 0.001f;    // Process noise  
    float Q_bias = 0.003f;     // Process noise bias
    float R_measure = 0.03f;   // Measurement noise
    
    // Stati Kalman [Roll, Pitch, Yaw]
    float angle[3] = {0};    
    float bias[3] = {0};     
    float P[3][2][2];        // Matrice covarianza
    
    // EMA (dal codice attuale)
    float ema_alpha = 0.1f;
    float last_roll = 0.0f;
    float last_pitch = 0.0f;
    float last_yaw = 0.0f;
    
    // Zone morte adattive
    float yaw_dead_zone = 0.15f;
    float pitch_dead_zone = 0.1f;
    uint32_t last_significant_change = 0;
};

// === DRIFT COMPENSATION (funzionalità chiave WIT9-proto) ===
struct DriftCompensation {
    bool enabled = true;
    float yaw_drift_rate = 0.0f;      // °/s
    float yaw_reference = 0.0f;
    uint32_t last_compensation = 0;
    
    // Storia per regressione lineare (5 min @ 1Hz)
    float yaw_history[300];    
    uint32_t history_timestamps[300];
    int history_index = 0;
    bool history_full = false;
    
    uint32_t reference_time = 0;
    const float MAX_COMP_RATE = 0.00033f; // ±0.02°/min max
};

// === STRUTTURA DATI PRINCIPALE ===
struct HWT906Data {
    // Dati base 
    float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
    float pitch_raw = 0.0f, yaw_raw = 0.0f, roll_raw = 0.0f;
    float pitch_filtered = 0.0f, yaw_filtered = 0.0f, roll_filtered = 0.0f;
    
    // Accelerometro/Giroscopio/Magnetometro raw
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    
    // Stato sistema
    bool is_static = false;
    uint32_t static_start_time = 0;
    float static_threshold = 0.5f;  // °/s
    
    // Performance tracking
    float update_rate = 0.0f;
    uint32_t frame_count = 0;
    uint32_t error_frames = 0;
    uint32_t last_frame_time = 0;
    
    // Range monitoring
    float pitch_min = 999.0f, pitch_max = -999.0f;
    float yaw_min = 999.0f, yaw_max = -999.0f;
    float roll_min = 999.0f, roll_max = -999.0f;
    
    // Calibrazione
    bool is_calibrated = false;
    float pitch_offset = 0.0f;
    float yaw_offset = 0.0f;
    float roll_offset = 0.0f;
};

// === CLASSE PRINCIPALE ===
class HWT906Handler {
private:
    HWT906Data currentData;
    AdvancedFilters filters;
    DriftCompensation drift;
    HardwareSerial* uart = nullptr;
    
    uint8_t frame_buffer[FRAME_SIZE];
    int frame_index = 0;
    bool frame_started = false;
    
    uint32_t last_valid_frame = 0;
    uint32_t debug_counter = 0;
    static const uint32_t DEBUG_INTERVAL = 50; // Debug ogni 50 frame
    
    HWT906PrecisionMode current_mode = HWT906_MODE_STATIC_MED;
    bool sensors_ready = false;
    
    // === ARRAY CONFIGURAZIONI (dal WIT9-proto) ===
    HWT906PrecisionConfig precision_configs[4] = {
        {0x0005, 0x0006, 0x0064, 0x07D0, 0x0005, 0x1388, "Static Low - Max Filter"},
        {0x0006, 0x0005, 0x0050, 0x05DC, 0x0008, 0x0FA0, "Static Med - Balanced"},    // DEFAULT
        {0x0007, 0x0004, 0x003C, 0x03E8, 0x000A, 0x0BB8, "Static High - Responsive"},
        {0x0009, 0x0003, 0x0028, 0x01F4, 0x0010, 0x07D0, "Dynamic - Test Only"}
    };

    // === METODI PRIVATI ===
    
    // I2C Configuration
    bool unlockHWT906();
    bool writeRegister(uint8_t reg, uint16_t value);
    bool lockHWT906();
    
    // UART Parsing - PARSING ROBUSTO dal WIT9-proto
    bool parseFrame(uint8_t* frame);
    bool validateFrame(uint8_t* frame);
    void processAngleFrame(uint8_t* frame);
    void processAccelFrame(uint8_t* frame);
    void processGyroFrame(uint8_t* frame);
    void processMagFrame(uint8_t* frame);
    
    // FILTRI MULTI-STADIO completi dal WIT9-proto
    void initAdvancedFilters();
    void applyAdvancedFiltering();
    void updateMovingAverage();
    void applyComplementaryFilter();
    void applyKalmanFilterYaw();
    void applyEMAFilter();
    void applyStaticThreshold();
    void resetAllFilters();
    
    // DRIFT COMPENSATION - funzionalità chiave
    void initDriftCompensation();
    void compensateDrift();
    void calculateDriftRate();
    void updateDriftHistory();
    void enableDriftCompensation(bool enable);
    
    // RILEVAMENTO STATO STATICO
    void detectStaticState();
    
    // Calibrazione
    void loadCalibration();
    void saveCalibration();
    
    // Debug controllato
    void printDebugInfo();
    void updateStatistics();

public:
    // === API PUBBLICA ===
    
    // Inizializzazione
    bool init(HardwareSerial* serial_port = &Serial2);
    bool configureStaticOptimized(HWT906PrecisionMode mode = HWT906_MODE_STATIC_MED);
    
    // Acquisizione dati
    void update();  // Da chiamare nel main loop a 10Hz
    bool parseUARTData();
    
    // Getters principali (compatibili con API esistente)
    float getPitch() { return currentData.pitch_filtered; }
    float getYaw() { return currentData.yaw_filtered; }
    float getRoll() { return currentData.roll_filtered; }
    
    // Getters aggiuntivi
    float getPitchRaw() { return currentData.pitch_raw; }
    float getYawRaw() { return currentData.yaw_raw; }
    float getRollRaw() { return currentData.roll_raw; }
    
    // Stato sistema
    bool isReady() { return sensors_ready; }
    bool isStatic() { return currentData.is_static; }
    float getUpdateRate() { return currentData.update_rate; }
    uint32_t getFrameCount() { return currentData.frame_count; }
    uint32_t getErrorFrames() { return currentData.error_frames; }
    
    // Calibrazione
    void startCalibration();
    void saveCurrentAsZero();
    void resetCalibration();
    bool isCalibrated() { return currentData.is_calibrated; }
    
    // Configurazione
    void setPrecisionMode(HWT906PrecisionMode mode);
    void setDriftCompensation(bool enable);
    void setDeadZones(float pitch_zone, float yaw_zone);
    
    // Range info
    void getRange(float& pitch_min, float& pitch_max, float& yaw_min, float& yaw_max);
    void resetRange();
    
    // Debug
    void printStatus();
    void enableDebug(bool enable);
    
    // Struttura dati completa (per debugging avanzato)
    const HWT906Data& getData() const { return currentData; }
};

// === FUNZIONI GLOBALI COMPATIBILI ===
extern HWT906Handler hwt906;

// API wrapper per compatibilità con codice esistente
bool initHWT906();
void updateHWT906();
float getHWT906Pitch();
float getHWT906Yaw();
float getHWT906Roll();
bool isHWT906Ready();

#endif // HWT906_HANDLER_H
