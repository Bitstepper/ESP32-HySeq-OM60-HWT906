// hwt906_handler_improved.cpp
// Versione migliorata con filtri avanzati e compensazione drift da WIT9-proto.ino

#include "hwt906_handler.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <Preferences.h>

// === COSTANTI FILTRI AVANZATI ===
#define MOVING_AVG_SIZE 10
#define COMPLEMENTARY_ALPHA 0.98f  // Per uso statico
#define STATIC_THRESHOLD 0.01f     // Soglia freeze
#define YAW_DRIFT_WINDOW 300       // 5 minuti @ 1Hz

// Forward declarations
bool configureStaticOptimized(PrecisionMode mode);
void processFrame(uint8_t* frame);
void applyAdvancedFiltering();
void compensateDrift();
void calculateDriftRate();
bool unlockHWT906();
bool writeRegister(uint8_t reg, uint16_t value);

// === STRUTTURE DATI (spostate nel header per accesso esterno) ===

// === MODALITÀ PRECISIONE (da WIT9-proto) ===

struct PrecisionConfig {
    uint16_t output_rate;      
    uint16_t bandwidth;        
    float ema_alpha;          
    float static_threshold;   
    const char* description;
};

// Configurazioni ottimizzate per misure statiche
PrecisionConfig precision_configs[] = {
    {0x0005, 0x0006, 0.95f, 0.005f, "Static Low - Max Filter"},    
    {0x0006, 0x0005, 0.90f, 0.010f, "Static Med - Balanced"},      
    {0x0007, 0x0004, 0.85f, 0.015f, "Static High - Responsive"},
    {0x0009, 0x0003, 0.70f, 0.050f, "Dynamic - Comparison"}        
};

// === VARIABILI GLOBALI ===
// Non static per permettere accesso da hwt906_compat.h
HWT906Data currentData = {};
AdvancedFilters filters = {};
DriftCompensation drift_comp = {};
static PrecisionMode current_precision = PRECISION_STATIC_MEDIUM;

static HardwareSerial* hwt906Serial = nullptr;
static Preferences prefs;

// === INIZIALIZZAZIONE MIGLIORATA ===
bool initHWT906() {
    Serial.println("🔧 Inizializzazione HWT906 con filtri avanzati...");
    
    // Reset strutture
    memset(&currentData, 0, sizeof(currentData));
    memset(&filters, 0, sizeof(filters));
    memset(&drift_comp, 0, sizeof(drift_comp));
    
    // Init Kalman
    filters.kalman_P = 1.0f;
    filters.kalman_Q = 0.001f;  // Process noise
    filters.kalman_R = 0.1f;    // Measurement noise
    
    // Abilita drift compensation
    drift_comp.enabled = true;
    
    // Carica calibrazione
    prefs.begin("hwt906", false);
    currentData.pitch_offset = prefs.getFloat("pitch_off", 0);
    currentData.yaw_offset = prefs.getFloat("yaw_off", 0);
    prefs.end();
    
    // I2C per configurazione
    Wire.beginTransmission(HWT906_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("❌ HWT906 non trovato su I2C!");
        return false;
    }
    
    // UART per dati
    if (!hwt906Serial) {
        hwt906Serial = new HardwareSerial(2);
    }
    hwt906Serial->begin(9600, SERIAL_8N1, HWT906_RX_PIN, HWT906_TX_PIN);
    hwt906Serial->setTimeout(10);
    
    // Configurazione ottimizzata
    if (!configureStaticOptimized(current_precision)) {
        return false;
    }
    
    currentData.sensor_present = true;
    Serial.println("✅ HWT906 inizializzato con successo!");
    Serial.printf("📊 Modo: %s\n", precision_configs[current_precision].description);
    
    return true;
}

// === CONFIGURAZIONE AVANZATA ===
bool configureStaticOptimized(PrecisionMode mode) {
    Serial.printf("⚙️ Configurazione HWT906 modo %d...\n", mode);
    
    PrecisionConfig& config = precision_configs[mode];
    
    // Unlock registro
    if (!unlockHWT906()) {
        return false;
    }
    
    bool success = true;
    
    // 1. Reset completo
    success &= writeRegister(0x01, 0x0001);
    delay(500);
    
    // 2. Unlock di nuovo dopo reset
    if (!unlockHWT906()) {
        return false;
    }
    
    // 3. Range accelerometro ±2g
    success &= writeRegister(0x21, 0x0000);
    delay(100);
    
    // 4. Range giroscopio ±2000°/s  
    success &= writeRegister(0x20, 0x0003);
    delay(100);
    
    // 5. CRITICO: Abilita calcolo angoli automatico
    success &= writeRegister(0x24, 0x0001);  // Algoritmo ON
    delay(100);
    
    // 6. Modalità output continuo
    success &= writeRegister(0x23, 0x0001);  // Auto send
    delay(100);
    
    // 7. Output rate
    success &= writeRegister(0x03, config.output_rate);
    delay(100);
    
    // 8. Bandwidth
    success &= writeRegister(0x1F, config.bandwidth);
    delay(100);
    
    // 9. SALVA configurazione
    success &= writeRegister(0x00, 0x0000);
    delay(500);
    
    if (success) {
        current_precision = mode;
        Serial.println("✅ Configurazione angoli completata");
    } else {
        Serial.println("❌ Errore configurazione");
    }
    
    return success;
}


// === PARSING MIGLIORATO === MUTUATO DA WIT9-Proto.ino (il main allinone)
void parseHWT906Data() {
    static uint8_t buffer[64];
    static uint8_t bufferIndex = 0;

    
    // Debug periodico
    static uint32_t lastDebug = 0;
/*    if (millis() - lastDebug > 1000) {
        Serial.printf("UART: Available=%d, Buffer=%d, Frames=%lu\n", 
                     hwt906Serial ? hwt906Serial->available() : -1, 
                     bufferIndex,
                     currentData.frames_received);
        lastDebug = millis();
    }
*/
    
    // Accumula TUTTI i byte disponibili
    while (hwt906Serial && hwt906Serial->available() && bufferIndex < 64) {
        buffer[bufferIndex++] = hwt906Serial->read();
    }
    
    // Cerca frame validi quando hai abbastanza dati
    if (bufferIndex >= 11) {
        for (int i = 0; i <= bufferIndex - 11; i++) {
            if (buffer[i] == 0x55) {
                // Verifica checksum
                uint8_t checksum = 0;
                for (int j = 0; j < 10; j++) {
                    checksum += buffer[i + j];
                }
                
if (checksum == buffer[i + 10]) {
    // Frame valido trovato!
//    Serial.printf("Frame OK! Type=0x%02X at pos %d\n", buffer[i+1], i);
    processFrame(&buffer[i]);
    currentData.frames_received++;
    currentData.last_update = millis();  // ← AGGIUNGI QUESTA RIGA

    // Debug aggiuntivo
//    Serial.printf("✅ HWT906 ora PRESENTE! Last update: %lu\n", currentData.last_update);
    
    // Rimuovi frame processato dal buffer
    int remaining = bufferIndex - (i + 11);
    if (remaining > 0) {
        memmove(buffer, &buffer[i + 11], remaining);
    }
    bufferIndex = remaining;
    break;  // Processa un frame alla volta
}

            }
        }
        
        // Se il buffer è pieno e non ci sono frame validi, mantieni ultimi 10 byte
        if (bufferIndex >= 64) {
            memmove(buffer, &buffer[bufferIndex - 10], 10);
            bufferIndex = 10;
        }
    }
    
    // Update rate
    static uint32_t lastRateCalc = 0;
    if (millis() - lastRateCalc > 1000) {
        currentData.update_rate = currentData.frames_received - 
                                 currentData.last_frame_count;
        currentData.last_frame_count = currentData.frames_received;
        lastRateCalc = millis();
    }
}

// === ELABORAZIONE FRAME ===
void processFrame(uint8_t* frame) {
    uint8_t frameType = frame[1];
    
    switch (frameType) {
        case 0x51: // Accelerazione
		// CORRETTO (low byte first)
		currentData.ax = (int16_t)(frame[2] | (frame[3] << 8)) / 32768.0f * 2.0f;
		currentData.ay = (int16_t)(frame[4] | (frame[5] << 8)) / 32768.0f * 2.0f;
		currentData.az = (int16_t)(frame[6] | (frame[7] << 8)) / 32768.0f * 2.0f;
            break;
            
        case 0x52: // Giroscopio
		// CORRETTO (low byte first)  
		currentData.gx = (int16_t)(frame[2] | (frame[3] << 8)) / 32768.0f * 2000.0f;
		currentData.gy = (int16_t)(frame[4] | (frame[5] << 8)) / 32768.0f * 2000.0f;
		currentData.gz = (int16_t)(frame[6] | (frame[7] << 8)) / 32768.0f * 2000.0f;
            
            // Calcola magnitudine per rilevamento statico
            filters.last_gyro_magnitude = sqrt(currentData.gx * currentData.gx + 
                                             currentData.gy * currentData.gy + 
                                             currentData.gz * currentData.gz);
            break;
            

case 0x53: // Angoli
    // *** DEBUG FRAME RAW ***
/*    Serial.printf("📦 FRAME 0x53: [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]\n", 
                  frame[0], frame[1], frame[2], frame[3], frame[4], 
                  frame[5], frame[6], frame[7], frame[8], frame[9], frame[10]);
*/
    
    // Debug byte specifici degli angoli
 /*   Serial.printf("🎯 ANGLE BYTES: frame[4]=0x%02X frame[5]=0x%02X (Pitch), frame[6]=0x%02X frame[7]=0x%02X (Yaw)\n",
                  frame[4], frame[5], frame[6], frame[7]);
 */   
    // Calcolo con debug dei valori intermedi
    int16_t pitch_raw_int = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t yaw_raw_int = (int16_t)(frame[6] | (frame[7] << 8));
    
//    Serial.printf("🔢 INT16: Pitch=%d, Yaw=%d\n", pitch_raw_int, yaw_raw_int);
    
    // CORRETTO (low byte first)
    float new_pitch = pitch_raw_int / 32768.0f * 180.0f;
    float new_yaw = yaw_raw_int / 32768.0f * 180.0f;
 
/*    
    // *** DEBUG CALCOLI ***
    Serial.printf("🔍 RAW: P=%.2f° Y=%.2f° | Offset: P=%.2f° Y=%.2f°\n", 
                  new_pitch, new_yaw, 
                  currentData.pitch_offset, currentData.yaw_offset);
*/
    
    // Applica offset calibrazione
    new_pitch -= currentData.pitch_offset;
    new_yaw -= currentData.yaw_offset;
    
/*
    Serial.printf("🎯 AFTER OFFSET: P=%.2f° Y=%.2f°\n", new_pitch, new_yaw);            
*/
    
    // Salva valori raw
    currentData.pitch_raw = new_pitch;
    currentData.yaw_raw = new_yaw;
 
/*    
    Serial.printf("💾 SAVED: P_raw=%.2f° Y_raw=%.2f°\n", 
                  currentData.pitch_raw, currentData.yaw_raw);            
*/
    
    // Applica filtri avanzati
    applyAdvancedFiltering();

/*    
    Serial.printf("✨ FILTERED: P=%.2f° Y=%.2f°\n", 
                  currentData.pitch, currentData.yaw);            
*/
    

    currentData.valid = true;
    currentData.last_update = millis();
    break;

    }
}

// === FILTRI AVANZATI MULTI-STADIO ===
void applyAdvancedFiltering() {
    // 1. MOVING AVERAGE
    filters.pitch_buffer[filters.buffer_index] = currentData.pitch_raw;
    filters.yaw_buffer[filters.buffer_index] = currentData.yaw_raw;
    
    filters.buffer_index = (filters.buffer_index + 1) % MOVING_AVG_SIZE;
    if (filters.buffer_index == 0) {
        filters.buffer_filled = true;
    }
    
    // Calcola media
    float pitch_avg = 0, yaw_avg = 0;
    int samples = filters.buffer_filled ? MOVING_AVG_SIZE : filters.buffer_index;
    
    for (int i = 0; i < samples; i++) {
        pitch_avg += filters.pitch_buffer[i];
        yaw_avg += filters.yaw_buffer[i];
    }
    pitch_avg /= samples;
    yaw_avg /= samples;
    
    // 2. FILTRO COMPLEMENTARE (solo se giroscopio stabile)
    if (filters.last_gyro_magnitude < 0.5f) {
        filters.comp_pitch = COMPLEMENTARY_ALPHA * filters.comp_pitch + 
                            (1.0f - COMPLEMENTARY_ALPHA) * pitch_avg;
        filters.comp_yaw = COMPLEMENTARY_ALPHA * filters.comp_yaw + 
                          (1.0f - COMPLEMENTARY_ALPHA) * yaw_avg;
    } else {
        // In movimento, usa direttamente media
        filters.comp_pitch = pitch_avg;
        filters.comp_yaw = yaw_avg;
    }
    
    // 3. KALMAN PER YAW (semplificato)
    // Predict
    filters.kalman_P += filters.kalman_Q;
    
    // Update
    float K = filters.kalman_P / (filters.kalman_P + filters.kalman_R);
    filters.kalman_yaw += K * (filters.comp_yaw - filters.kalman_yaw);
    filters.kalman_P = (1.0f - K) * filters.kalman_P;
    
    // 4. RILEVAMENTO STATO STATICO
    if (filters.last_gyro_magnitude < 0.1f) {
        if (!filters.is_static) {
            filters.static_start_time = millis();
            filters.is_static = true;
        }
    } else {
        filters.is_static = false;
    }
    
    // 5. SOGLIA STATICA (freeze sotto threshold)
    float pitch_change = fabs(filters.comp_pitch - currentData.pitch);
    float yaw_change = fabs(filters.kalman_yaw - currentData.yaw);
    
    // Se statico da più di 2 secondi, applica soglia più stretta
    float threshold = STATIC_THRESHOLD;
    if (filters.is_static && (millis() - filters.static_start_time > 2000)) {
        threshold = STATIC_THRESHOLD / 2.0f;
    }
    
    // Aggiorna solo se sopra soglia
    if (pitch_change > threshold) {
        currentData.pitch = filters.comp_pitch;
    }
    
    if (yaw_change > threshold) {
        currentData.yaw = filters.kalman_yaw;
    }
    
    // 6. COMPENSAZIONE DRIFT YAW
    compensateDrift();
}

// === COMPENSAZIONE DRIFT ===
void compensateDrift() {
    if (!drift_comp.enabled) return;
    
    uint32_t now = millis();
    
    // Aggiungi campione ogni secondo
    static uint32_t last_sample = 0;
    if (now - last_sample > 1000) {
        drift_comp.yaw_history[drift_comp.history_index] = currentData.yaw;
        drift_comp.timestamps[drift_comp.history_index] = now;
        
        drift_comp.history_index = (drift_comp.history_index + 1) % YAW_DRIFT_WINDOW;
        if (drift_comp.history_index == 0) {
            drift_comp.history_filled = true;
        }
        
        last_sample = now;
        
        // Calcola drift rate se abbiamo abbastanza dati
        if (drift_comp.history_filled || drift_comp.history_index > 60) {
            calculateDriftRate();
        }
    }
    
    // Applica compensazione
    if (drift_comp.yaw_drift_rate != 0 && filters.is_static) {
        float dt = (now - drift_comp.last_compensation_time) / 1000.0f;
        float compensation = drift_comp.yaw_drift_rate * dt;
        
        // Limita compensazione
        compensation = constrain(compensation, -0.02f, 0.02f);
        
        currentData.yaw -= compensation;
        drift_comp.yaw_accumulated += compensation;
        
        drift_comp.last_compensation_time = now;
    }
}

// === CALCOLO DRIFT RATE ===
void calculateDriftRate() {
    int samples = drift_comp.history_filled ? YAW_DRIFT_WINDOW : drift_comp.history_index;
    if (samples < 30) return;  // Minimo 30 secondi
    
    // Regressione lineare semplice
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    float t0 = drift_comp.timestamps[0] / 1000.0f;
    
    for (int i = 0; i < samples; i++) {
        float x = (drift_comp.timestamps[i] / 1000.0f) - t0;
        float y = drift_comp.yaw_history[i];
        
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }
    
    float n = samples;
    float slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    
    // Aggiorna drift rate (°/s)
    drift_comp.yaw_drift_rate = slope;
    
    // Limita a valori ragionevoli
    drift_comp.yaw_drift_rate = constrain(drift_comp.yaw_drift_rate, -0.00033f, 0.00033f);
}

// === CALIBRAZIONE MIGLIORATA ===
void calibrateHWT906() {
    Serial.println("🎯 Calibrazione HWT906 (20 secondi)...");
    
    float pitch_sum = 0, yaw_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;
    
    uint32_t start_time = millis();
    
    while (millis() - start_time < 20000) {
        parseHWT906Data();
        
        if (currentData.valid) {
            pitch_sum += currentData.pitch_raw;
            yaw_sum += currentData.yaw_raw;
            gx_sum += currentData.gx;
            gy_sum += currentData.gy;
            gz_sum += currentData.gz;
            samples++;
        }
        
        delay(10);
    }
    
    if (samples > 0) {
        // Calcola offset
        currentData.pitch_offset = pitch_sum / samples;
        currentData.yaw_offset = yaw_sum / samples;
        
        // Calcola bias giroscopio
        float gx_bias = gx_sum / samples;
        float gy_bias = gy_sum / samples;
        float gz_bias = gz_sum / samples;
        
        // Salva calibrazione
        prefs.begin("hwt906", false);
        prefs.putFloat("pitch_off", currentData.pitch_offset);
        prefs.putFloat("yaw_off", currentData.yaw_offset);
        prefs.end();
        
        // Reset filtri
        memset(&filters, 0, sizeof(filters));
        filters.kalman_P = 1.0f;
        filters.kalman_Q = 0.001f;
        filters.kalman_R = 0.1f;
        
        // Reset drift compensation
        memset(&drift_comp, 0, sizeof(drift_comp));
        drift_comp.enabled = true;
        
        Serial.println("✅ Calibrazione completata!");
        Serial.printf("📐 Offset: Pitch=%.2f° Yaw=%.2f°\n", 
                     currentData.pitch_offset, currentData.yaw_offset);
        Serial.printf("🌀 Bias giroscopio: X=%.3f Y=%.3f Z=%.3f °/s\n",
                     gx_bias, gy_bias, gz_bias);
    }
}

// === FUNZIONI DI ACCESSO ===
float getHWT906Pitch() {
    return currentData.pitch;
}

float getHWT906Yaw() {
    return currentData.yaw;
}

bool isHWT906Present() {
    return currentData.sensor_present && 
           (millis() - currentData.last_update < 1000);
}

// === FUNZIONI DI UTILITÀ ===
void setDriftCompensation(bool enabled) {
    drift_comp.enabled = enabled;
    if (enabled) {
        Serial.println("✅ Compensazione drift abilitata");
    } else {
        Serial.println("❌ Compensazione drift disabilitata");
    }
}

void setPrecisionMode(PrecisionMode mode) {
    if (mode != current_precision) {
        configureStaticOptimized(mode);
    }
}

void getStabilityMetrics(float& pitch_sigma, float& yaw_sigma) {
    // Calcola deviazione standard ultimi 10 campioni
    if (!filters.buffer_filled && filters.buffer_index < 2) {
        pitch_sigma = yaw_sigma = 0;
        return;
    }
    
    int samples = filters.buffer_filled ? MOVING_AVG_SIZE : filters.buffer_index;
    
    // Media
    float pitch_mean = 0, yaw_mean = 0;
    for (int i = 0; i < samples; i++) {
        pitch_mean += filters.pitch_buffer[i];
        yaw_mean += filters.yaw_buffer[i];
    }
    pitch_mean /= samples;
    yaw_mean /= samples;
    
    // Deviazione standard
    float pitch_var = 0, yaw_var = 0;
    for (int i = 0; i < samples; i++) {
        pitch_var += pow(filters.pitch_buffer[i] - pitch_mean, 2);
        yaw_var += pow(filters.yaw_buffer[i] - yaw_mean, 2);
    }
    
    pitch_sigma = sqrt(pitch_var / samples);
    yaw_sigma = sqrt(yaw_var / samples);
}

// === HELPER FUNCTIONS ===
bool unlockHWT906() {
    for (int retry = 0; retry < 3; retry++) {
        Wire.beginTransmission(HWT906_I2C_ADDR);
        Wire.write(0xFF);
        Wire.write(0xAA);
        Wire.write(0x69);
        Wire.write(0x88);
        Wire.write(0xB5);
        
        if (Wire.endTransmission() == 0) {
            delay(100);
            return true;
        }
        delay(200);
    }
    return false;
}

bool writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(HWT906_I2C_ADDR);
    Wire.write(0xFF);
    Wire.write(0xAA);
    Wire.write(reg);
    Wire.write(value & 0xFF);
    Wire.write((value >> 8) & 0xFF);
    
    return Wire.endTransmission() == 0;
}

// === FUNZIONI STUB PER COMPATIBILITÀ MAG ===
// Il HWT906 ha il magnetometro ma la calibrazione è diversa
void startMagCalibration() {
    Serial.println("⚠️ Calibrazione magnetometro HWT906 non implementata");
    // TODO: Implementare calibrazione mag specifica per HWT906
}

bool isMagCalibrationInProgress() {
    return false;  // Per ora sempre false
}

float getMagCalibrationProgress() {
    return 0.0f;  // Per ora sempre 0%
}

void finishMagCalibration() {
    // Nulla da fare per ora
}
