// hwt906_handler.h
#ifndef HWT906_HANDLER_H
#define HWT906_HANDLER_H

#include <Arduino.h>
#include <HardwareSerial.h>

// === CONFIGURAZIONE HARDWARE ===
#define HWT906_I2C_ADDR     0x50    // Indirizzo I2C per configurazione
#define HWT906_RX_PIN       44      // UART RX pin
#define HWT906_TX_PIN       43      // UART TX pin
#define HWT906_BAUD         9600    // UART baud rate fisso

// === PROTOCOLLO WITMOTION ===
#define FRAME_HEADER        0x55
#define TYPE_TIME          0x50
#define TYPE_ACC           0x51  
#define TYPE_GYRO          0x52
#define TYPE_ANGLE         0x53
#define TYPE_MAG           0x54
#define TYPE_PORT          0x55
#define TYPE_QUAT          0x59

// === CONFIGURAZIONE FILTRI ===
#define HWT906_EMA_ALPHA    0.2f     // Filtro EMA (0.2 = 80% vecchio, 20% nuovo)
#define HWT906_YAW_DEADZONE 0.15f    // Zona morta yaw
#define HWT906_PITCH_DEADZONE 0.1f   // Zona morta pitch

// === MODALITÀ PRECISIONE ===
enum HWT906PrecisionMode {
    HWT906_MODE_STATIC_HIGH = 0,    // 5Hz, filtri massimi
    HWT906_MODE_STATIC_MED = 1,     // 10Hz, filtri bilanciati  
    HWT906_MODE_DYNAMIC = 2,        // 50Hz, filtri minimi
    HWT906_MODE_RAW = 3             // 100Hz, no filtri
};

// === STRUTTURA DATI ===
struct HWT906Data {
    // Angoli principali (quello che serve al display)
    float pitch;            // -90 to +90°
    float yaw;              // -180 to +180°
    float roll;             // -180 to +180°
    
    // Angoli filtrati
    float pitch_filtered;
    float yaw_filtered;
    float roll_filtered;
    
    // Dati grezzi disponibili
    float ax, ay, az;       // Accelerazioni (g)
    float gx, gy, gz;       // Velocità angolari (°/s)
    float hx, hy, hz;       // Campo magnetico
    float q0, q1, q2, q3;   // Quaternioni
    
    // Temperatura e stato
    float temperature;      // °C
    bool valid;            // Dati validi
    bool sensor_present;   // Sensore rilevato
    uint32_t timestamp;    // ms
    
    // Statistiche comunicazione
    uint32_t frames_received;
    uint32_t frames_error;
    uint32_t last_frame_time;
    float update_rate_hz;
    
    // Stato calibrazione
    bool is_calibrated;
    float pitch_offset;
    float yaw_offset;
    float roll_offset;
    
    // Drift compensation
    bool drift_compensation_enabled;
    float yaw_drift_rate;   // °/min
    uint32_t static_time_ms;
    bool is_static;
};

// === API PUBBLICA (compatibile con imu_handler) ===
bool initHWT906();
bool isHWT906Ready();
bool isHWT906Present();

// Lettura dati completa
HWT906Data getHWT906Data();
void updateHWT906();  // Da chiamare nel loop

// Lettura singoli assi (compatibilità)
float getHWT906Pitch();
float getHWT906Yaw();
float getHWT906Roll();

// Calibrazione
void calibrateHWT906();
void zeroHWT906Angles();
bool saveHWT906Calibration();
bool loadHWT906Calibration();

// Funzioni per leaf_actions.cpp
void startMagCalibration();
bool isMagCalibrationInProgress();
float getMagCalibrationProgress();
void finishMagCalibration();

// Configurazione
bool configureHWT906Mode(HWT906PrecisionMode mode);
void setHWT906Filters(float pitch_alpha, float yaw_alpha);
void enableHWT906DriftCompensation(bool enable);

// Diagnostica
void printHWT906Debug();
void printHWT906Config();
float getHWT906UpdateRate();
bool testHWT906Communication();

// Reset
void resetHWT906();
void restartHWT906();

#endif // HWT906_HANDLER_H