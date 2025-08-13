// om60_handler.h
#ifndef OM60_HANDLER_H
#define OM60_HANDLER_H

#include <Arduino.h>

// === CONFIGURAZIONE HARDWARE ESP32-S3 ===
#define OM60_ADC_PIN        15      // GPIO15 disponibile su board
#define OM60_ADC_CHANNEL    ADC1_CHANNEL_4  // GPIO15 = CH4
#define OM60_ADC_ATTEN      ADC_ATTEN_DB_12  // 0-3.3V range (11dB deprecato)
#define OM60_ADC_WIDTH      ADC_WIDTH_BIT_12

// === OVERSAMPLING PER 14-BIT EFFETTIVI ===
#define OM60_OVERSAMPLING   16      // 16x = +2 bit (12+2=14 bit)
#define OM60_SAMPLES_BURST  4       // 4 burst da 16 = 64 totali

// === CALIBRAZIONE HARDWARE ===
#define OM60_VDIV_RATIO     0.270f  // 10k/(27k+10k) 
#define OM60_VREF_ACTUAL    3.30f   // Misurare con multimetro!
#define OM60_ADC_MAX        4095.0f

// === PARAMETRI SENSORE ===
#define OM60_RANGE_MIN      200     // mm
#define OM60_RANGE_MAX      1000    // mm
#define OM60_VOLTAGE_SPAN   10.0f   // 0-10V
#define OM60_DISTANCE_SPAN  800.0f  // 800mm

// === FILTRI ===
#define OM60_MEDIAN_SIZE    5       // Filtro mediano 5 campioni
#define OM60_EMA_ALPHA      0.3f    // Filtro EMA finale

struct OM60CalibrationData {
    // Punti calibrazione (minimo 2, meglio 5)
    struct CalPoint {
        float reference_mm;     // Distanza riferimento
        float voltage_measured; // Tensione misurata
        uint16_t adc_raw;      // Valore ADC
        bool valid;
    } points[5];
    
    // Coefficienti regressione lineare
    float slope;              // mm/V
    float offset;             // mm
    float r_squared;          // Qualità fit
    
    // Correzione non-linearità
    float linearity_error[10]; // Errore ogni 100mm
};

struct OM60Data {
    // Misure principali
    float distance_mm;          // Distanza finale filtrata
    float distance_raw_mm;      // Distanza senza filtri
    float voltage_input;        // Tensione stimata ingresso (0-10V)
    float voltage_adc;          // Tensione ADC (0-2.7V)
    
    // Dati ADC
    uint16_t adc_raw;          // Singola lettura
    uint32_t adc_averaged;     // Dopo oversampling (14-bit)
    uint16_t adc_min;          // Min in buffer
    uint16_t adc_max;          // Max in buffer
    
    // Qualità misura
    float noise_pp;            // Rumore picco-picco in mm
    float noise_rms;           // Rumore RMS in mm
    float stability_score;     // 0-100% stabilità
    
    // Stato
    bool valid;
    bool in_range;
    bool sensor_present;       // Per compatibilità con leaf_actions
    uint32_t timestamp;
    uint32_t sample_count;
};

// === API PUBBLICA (compatibile con radar_handler) ===
bool initOM60();
bool isOM60Ready();
bool isOM60Present();         // Per leaf_actions
void updateOM60();            // Da chiamare a 10Hz

// Lettura distanza (compatibilità con radar_handler)
float getOM60Distance();              // Distanza grezza
float getFilteredOM60Distance();      // Distanza filtrata
OM60Data getOM60Data();              // Dati completi

// Calibrazione
bool calibrateOM60Point(uint8_t index, float reference_mm);
bool calculateOM60Calibration();
void saveOM60Calibration();
bool loadOM60Calibration();

// Configurazione
void setOM60FilterMode(uint8_t mode); // 0=OFF, 1=LIGHT, 2=MEDIUM, 3=HEAVY
void setOM60UpdateRate(uint16_t ms);  // Default 100ms
void setOM60AveragingSamples(uint8_t samples);

// Diagnostica
void printOM60Debug();
void runOM60SelfTest();
float getOM60Resolution();           // Risoluzione effettiva in mm
void plotOM60Noise(uint16_t samples); // Per Serial Plotter

// Funzioni per supporto calibrazione magnetometro (stub per compatibilità)
void startMagCalibration();
bool isMagCalibrationInProgress();
float getMagCalibrationProgress();
void finishMagCalibration();

#endif // OM60_HANDLER_H
