// hwt906_handler.cpp
#include "hwt906_handler.h"
#include <Wire.h>
//#include <EEPROM.h>  - rimosso e sostituito da Preferences.h
#include <Preferences.h>


// === AGGIUNGI QUESTE DICHIARAZIONI DOPO GLI INCLUDE IN hwt906_handler.cpp ===

// Forward declarations
static bool testI2CPresence();
static void parseUARTByte(uint8_t byte);
static bool validateFrame(uint8_t* frame);
static void processFrame(uint8_t* frame);
static void applyFilters();
static void detectStaticState();
static void applyDriftCompensation();
static bool configureBasicSettings();
static bool unlockHWT906();
static bool writeRegister(uint8_t reg, uint16_t value);
static void updateStatistics();

// === OGGETTI GLOBALI ===
static HardwareSerial hwt906Serial(2);  // UART2
static HWT906Data currentData = {0};
static bool sensorInitialized = false;
static uint32_t initAttempts = 0;
// Aggiungi questa variabile globale:
static Preferences preferences;

// Buffer parsing UART
static uint8_t rx_buffer[11];
static uint8_t buffer_index = 0;

// Filtri
static float pitch_ema = 0;
static float yaw_ema = 0;
static float roll_ema = 0;
static bool first_reading = true;

// Timer per verifiche non bloccanti
static uint32_t lastCheckTime = 0;
static uint32_t lastFrameTime = 0;
static const uint32_t TIMEOUT_MS = 2000;  // 2 secondi timeout

// Stato calibrazione magnetometro
static bool mag_calibration_active = false;
static uint32_t mag_calibration_start = 0;
static const uint32_t MAG_CALIBRATION_DURATION = 20000; // 20 secondi

// === INIZIALIZZAZIONE NON BLOCCANTE ===
bool initHWT906() {
    Serial.println("\n🔧 Inizializzazione HWT906...");
    
    // Reset stato
    memset(&currentData, 0, sizeof(currentData));
    sensorInitialized = false;
    initAttempts++;
    
    // Configura UART
    hwt906Serial.begin(HWT906_BAUD, SERIAL_8N1, HWT906_RX_PIN, HWT906_TX_PIN);
    hwt906Serial.setTimeout(100);  // Timeout breve per non bloccare
    
    // Test presenza sensore via I2C (non bloccante)
    Serial.println("🔍 Ricerca HWT906 su I2C...");
    bool i2cFound = testI2CPresence();
    
    if (i2cFound) {
        Serial.println("✅ HWT906 trovato su I2C");
        currentData.sensor_present = true;
        
        // Tenta configurazione base
        if (configureBasicSettings()) {
            Serial.println("✅ Configurazione base applicata");
        } else {
            Serial.println("⚠️ Configurazione fallita, uso defaults");
        }
    } else {
        Serial.println("⚠️ HWT906 non trovato su I2C");
        currentData.sensor_present = false;
    }
    
    // Test UART (non bloccante)
    Serial.println("🔍 Test comunicazione UART...");
    lastCheckTime = millis();
    lastFrameTime = millis();
    
    // Svuota buffer
    while (hwt906Serial.available()) {
        hwt906Serial.read();
    }
    
    // Carica calibrazione se disponibile
    if (loadHWT906Calibration()) {
        Serial.println("✅ Calibrazione caricata da EEPROM");
        currentData.is_calibrated = true;
    }
    
    // Considera inizializzato anche se non presente (non bloccante)
    sensorInitialized = true;
    
    Serial.printf("📊 Init completato (tentativo %lu)\n", initAttempts);
    return true;
}

// === TEST PRESENZA I2C NON BLOCCANTE ===
bool testI2CPresence() {
    Wire.beginTransmission(HWT906_I2C_ADDR);
    uint8_t error = Wire.endTransmission();
    return (error == 0);
}

// === UPDATE PRINCIPALE NON BLOCCANTE ===
void updateHWT906() {
    if (!sensorInitialized) return;
    
    uint32_t now = millis();
    
    // Verifica timeout comunicazione
    if (now - lastFrameTime > TIMEOUT_MS) {
        if (currentData.sensor_present) {
            Serial.println("⚠️ HWT906 timeout - sensore disconnesso?");
            currentData.sensor_present = false;
            currentData.valid = false;
        }
        
        // Riprova connessione ogni 5 secondi
        if (now - lastCheckTime > 5000) {
            lastCheckTime = now;
            if (testI2CPresence()) {
                Serial.println("🔄 HWT906 riconnesso!");
                currentData.sensor_present = true;
                initHWT906();  // Re-init
            }
        }
    }
    
    // Parse dati UART se disponibili
    int bytesAvailable = hwt906Serial.available();
    if (bytesAvailable > 0) {
        // Limita letture per non bloccare
        int bytesToRead = min(bytesAvailable, 64);
        
        for (int i = 0; i < bytesToRead; i++) {
            parseUARTByte(hwt906Serial.read());
        }
    }
    
    // Aggiorna statistiche
    updateStatistics();
}

// === PARSING UART OTTIMIZZATO ===
void parseUARTByte(uint8_t byte) {
    // Cerca header
    if (byte == FRAME_HEADER && buffer_index == 0) {
        rx_buffer[buffer_index++] = byte;
    } 
    else if (buffer_index > 0) {
        rx_buffer[buffer_index++] = byte;
        
        // Frame completo
        if (buffer_index >= 11) {
            if (validateFrame(rx_buffer)) {
                processFrame(rx_buffer);
                currentData.frames_received++;
                lastFrameTime = millis();
                
                // Marca sensore come presente
                if (!currentData.sensor_present) {
                    currentData.sensor_present = true;
                    Serial.println("✅ HWT906 dati ricevuti!");
                }
            } else {
                currentData.frames_error++;
            }
            buffer_index = 0;
        }
    }
}

// === VALIDAZIONE FRAME ===
bool validateFrame(uint8_t* frame) {
    if (frame[0] != FRAME_HEADER) return false;
    
    uint8_t type = frame[1];
    if (type < 0x50 || type > 0x5A) return false;
    
    // Checksum
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += frame[i];
    }
    
    return (sum == frame[10]);
}

// === PROCESSING FRAME ===
void processFrame(uint8_t* frame) {
    uint8_t type = frame[1];
    
    switch (type) {
        case TYPE_ANGLE: {
            // Parsing angoli (i più importanti)
            int16_t roll_raw = (int16_t)(frame[2] | (frame[3] << 8));
            int16_t pitch_raw = (int16_t)(frame[4] | (frame[5] << 8));
            int16_t yaw_raw = (int16_t)(frame[6] | (frame[7] << 8));
            
            // Conversione a gradi
            float new_roll = roll_raw * (180.0f / 32768.0f);
            float new_pitch = pitch_raw * (180.0f / 32768.0f);
            float new_yaw = yaw_raw * (180.0f / 32768.0f);
            
            // Limiti fisici
            new_pitch = constrain(new_pitch, -90.0f, 90.0f);
            
            // Prima lettura: inizializza offset
            if (!currentData.valid) {
                currentData.roll_offset = new_roll;
                currentData.pitch_offset = new_pitch;
                currentData.yaw_offset = new_yaw;
                
                pitch_ema = 0;
                yaw_ema = 0;
                roll_ema = 0;
                first_reading = true;
                
                Serial.println("🎯 Offset iniziali impostati");
            }
            
            // Applica offset
            currentData.roll = new_roll - currentData.roll_offset;
            currentData.pitch = new_pitch - currentData.pitch_offset;
            currentData.yaw = new_yaw - currentData.yaw_offset;
            
            // Normalizza yaw
            while (currentData.yaw > 180.0f) currentData.yaw -= 360.0f;
            while (currentData.yaw < -180.0f) currentData.yaw += 360.0f;
            
            // Applica filtri
            applyFilters();
            
            currentData.valid = true;
            currentData.timestamp = millis();
            break;
        }
        
        case TYPE_ACC: {
            int16_t ax_raw = (int16_t)(frame[2] | (frame[3] << 8));
            int16_t ay_raw = (int16_t)(frame[4] | (frame[5] << 8));
            int16_t az_raw = (int16_t)(frame[6] | (frame[7] << 8));
            int16_t temp_raw = (int16_t)(frame[8] | (frame[9] << 8));
            
            currentData.ax = ax_raw / 32768.0f * 16.0f;  // ±16g range
            currentData.ay = ay_raw / 32768.0f * 16.0f;
            currentData.az = az_raw / 32768.0f * 16.0f;
            currentData.temperature = temp_raw / 340.0f + 36.25f;
            break;
        }
        
        case TYPE_GYRO: {
            int16_t gx_raw = (int16_t)(frame[2] | (frame[3] << 8));
            int16_t gy_raw = (int16_t)(frame[4] | (frame[5] << 8));
            int16_t gz_raw = (int16_t)(frame[6] | (frame[7] << 8));
            
            currentData.gx = gx_raw / 32768.0f * 2000.0f;  // ±2000°/s
            currentData.gy = gy_raw / 32768.0f * 2000.0f;
            currentData.gz = gz_raw / 32768.0f * 2000.0f;
            
            // Rileva stato statico
            detectStaticState();
            break;
        }
        
        case TYPE_MAG: {
            int16_t hx_raw = (int16_t)(frame[2] | (frame[3] << 8));
            int16_t hy_raw = (int16_t)(frame[4] | (frame[5] << 8));
            int16_t hz_raw = (int16_t)(frame[6] | (frame[7] << 8));
            
            currentData.hx = (float)hx_raw;
            currentData.hy = (float)hy_raw;
            currentData.hz = (float)hz_raw;
            break;
        }
    }
}

// === FILTRI ===
void applyFilters() {
    if (first_reading) {
        pitch_ema = currentData.pitch;
        yaw_ema = currentData.yaw;
        roll_ema = currentData.roll;
        first_reading = false;
    } else {
        // Filtro EMA
        pitch_ema = pitch_ema * (1 - HWT906_EMA_ALPHA) + currentData.pitch * HWT906_EMA_ALPHA;
        roll_ema = roll_ema * (1 - HWT906_EMA_ALPHA) + currentData.roll * HWT906_EMA_ALPHA;
        
        // Yaw con gestione wrap-around
        float yaw_diff = currentData.yaw - yaw_ema;
        if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
        if (yaw_diff < -180.0f) yaw_diff += 360.0f;
        yaw_ema += yaw_diff * HWT906_EMA_ALPHA;
        
        // Normalizza
        while (yaw_ema > 180.0f) yaw_ema -= 360.0f;
        while (yaw_ema < -180.0f) yaw_ema += 360.0f;
    }
    
    // Applica zona morta
    if (abs(pitch_ema - currentData.pitch_filtered) > HWT906_PITCH_DEADZONE) {
        currentData.pitch_filtered = pitch_ema;
    }
    
    if (abs(yaw_ema - currentData.yaw_filtered) > HWT906_YAW_DEADZONE) {
        currentData.yaw_filtered = yaw_ema;
    }
    
    currentData.roll_filtered = roll_ema;
    
    // Drift compensation per yaw se statico
    if (currentData.is_static && currentData.drift_compensation_enabled) {
        applyDriftCompensation();
    }
}

// === RILEVAMENTO STATO STATICO ===
void detectStaticState() {
    float gyro_magnitude = sqrt(currentData.gx * currentData.gx + 
                               currentData.gy * currentData.gy + 
                               currentData.gz * currentData.gz);
    
    bool is_static_now = (gyro_magnitude < 0.5f);  // 0.5°/s soglia
    
    if (is_static_now && !currentData.is_static) {
        currentData.is_static = true;
        currentData.static_time_ms = millis();
        Serial.println("🎯 Sistema STATICO rilevato");
    } else if (!is_static_now && currentData.is_static) {
        currentData.is_static = false;
        uint32_t static_duration = millis() - currentData.static_time_ms;
        Serial.printf("🏃 Sistema in MOVIMENTO (statico per %lu sec)\n", static_duration/1000);
    }
}

// === DRIFT COMPENSATION ===
void applyDriftCompensation() {
    static uint32_t drift_start_time = 0;
    static float yaw_at_start = 0;
    
    if (drift_start_time == 0) {
        drift_start_time = millis();
        yaw_at_start = currentData.yaw_filtered;
    }
    
    uint32_t elapsed = millis() - drift_start_time;
    if (elapsed > 60000) {  // Ogni minuto
        float drift = currentData.yaw_filtered - yaw_at_start;
        currentData.yaw_drift_rate = drift;  // °/min
        
        // Reset per prossima misura
        drift_start_time = millis();
        yaw_at_start = currentData.yaw_filtered;
        
        Serial.printf("📊 Drift rate: %.3f°/min\n", currentData.yaw_drift_rate);
    }
}

// === CONFIGURAZIONE I2C ===
bool configureBasicSettings() {
    // Unlock sensore
    if (!unlockHWT906()) {
        return false;
    }
    
    // Output: Angoli + Acc + Gyro
    writeRegister(0x02, 0x000E);  // 0x0E = ACC+GYRO+ANGLE
    
    // Rate: 10Hz per uso statico
    writeRegister(0x03, 0x0006);  // 10Hz
    
    // Baud: 9600 (default)
    writeRegister(0x04, 0x0002);
    
    // Algoritmo: 9-axis
    writeRegister(0x24, 0x0000);
    
    // Save
    writeRegister(0x00, 0x0000);
    
    delay(1000);  // Attendi save
    return true;
}

bool unlockHWT906() {
    Wire.beginTransmission(HWT906_I2C_ADDR);
    Wire.write(0xFF);
    Wire.write(0xAA);
    Wire.write(0x69);
    Wire.write(0x88);
    Wire.write(0xB5);
    return (Wire.endTransmission() == 0);
}

bool writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(HWT906_I2C_ADDR);
    Wire.write(0xFF);
    Wire.write(0xAA);
    Wire.write(reg);
    Wire.write(value & 0xFF);
    Wire.write((value >> 8) & 0xFF);
    return (Wire.endTransmission() == 0);
}

// === API PUBBLICA ===
bool isHWT906Ready() {
    return sensorInitialized && currentData.valid;
}

bool isHWT906Present() {
    return currentData.sensor_present;
}

HWT906Data getHWT906Data() {
    return currentData;
}

float getHWT906Pitch() {
    return currentData.pitch_filtered;
}

float getHWT906Yaw() {
    return currentData.yaw_filtered;
}

float getHWT906Roll() {
    return currentData.roll_filtered;
}

// === CALIBRAZIONE ===
void calibrateHWT906() {
    Serial.println("\n🎯 Calibrazione HWT906...");
    Serial.println("Mantieni il sensore FERMO per 5 secondi");
    
    // Accumula offset
    float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
    int samples = 0;
    
    uint32_t start = millis();
    while (millis() - start < 5000) {
        updateHWT906();
        if (currentData.valid) {
            sum_roll += currentData.roll;
            sum_pitch += currentData.pitch;
            sum_yaw += currentData.yaw;
            samples++;
        }
        delay(50);
    }
    
    if (samples > 0) {
        currentData.roll_offset = sum_roll / samples;
        currentData.pitch_offset = sum_pitch / samples;
        currentData.yaw_offset = sum_yaw / samples;
        currentData.is_calibrated = true;
        
        Serial.println("✅ Calibrazione completata!");
        Serial.printf("Offset: R=%.1f P=%.1f Y=%.1f\n", 
                      currentData.roll_offset, 
                      currentData.pitch_offset, 
                      currentData.yaw_offset);
        
        saveHWT906Calibration();
    } else {
        Serial.println("❌ Calibrazione fallita - nessun dato ricevuto");
    }
}

void zeroHWT906Angles() {
    if (currentData.valid) {
        currentData.roll_offset = currentData.roll;
        currentData.pitch_offset = currentData.pitch;
        currentData.yaw_offset = currentData.yaw;
        Serial.println("✅ Angoli azzerati");
    }
}

// === CALIBRAZIONE MAGNETOMETRO ===
void startMagCalibration() {
    mag_calibration_active = true;
    mag_calibration_start = millis();
    Serial.println("🧲 Calibrazione magnetometro avviata");
    
    // Comando per avviare calibrazione mag su HWT906
    if (currentData.sensor_present) {
        unlockHWT906();
        writeRegister(0x01, 0x0007);  // Start mag calibration
    }
}

bool isMagCalibrationInProgress() {
    return mag_calibration_active && 
           (millis() - mag_calibration_start < MAG_CALIBRATION_DURATION);
}

float getMagCalibrationProgress() {
    if (!mag_calibration_active) return 0.0f;
    
    uint32_t elapsed = millis() - mag_calibration_start;
    if (elapsed >= MAG_CALIBRATION_DURATION) {
        return 1.0f;
    }
    return (float)elapsed / MAG_CALIBRATION_DURATION;
}

void finishMagCalibration() {
    if (mag_calibration_active) {
        mag_calibration_active = false;
        
        // Salva calibrazione mag
        if (currentData.sensor_present) {
            unlockHWT906();
            writeRegister(0x00, 0x0000);  // Save calibration
        }
        
        Serial.println("🧲 Calibrazione magnetometro completata");
    }
}

// === DIAGNOSTICA ===
void printHWT906Debug() {
    Serial.println("\n=== HWT906 DEBUG ===");
    Serial.printf("Presenza: %s\n", currentData.sensor_present ? "SI" : "NO");
    Serial.printf("Dati validi: %s\n", currentData.valid ? "SI" : "NO");
    
    if (currentData.valid) {
        Serial.printf("Angoli raw: R=%.1f P=%.1f Y=%.1f\n", 
                      currentData.roll, currentData.pitch, currentData.yaw);
        Serial.printf("Angoli filtrati: R=%.1f P=%.1f Y=%.1f\n", 
                      currentData.roll_filtered, 
                      currentData.pitch_filtered, 
                      currentData.yaw_filtered);
        Serial.printf("Giroscopio: X=%.1f Y=%.1f Z=%.1f °/s\n", 
                      currentData.gx, currentData.gy, currentData.gz);
        Serial.printf("Accelerometro: X=%.2f Y=%.2f Z=%.2f g\n", 
                      currentData.ax, currentData.ay, currentData.az);
        Serial.printf("Temperatura: %.1f°C\n", currentData.temperature);
    }
    
    Serial.printf("Frame: RX=%lu ERR=%lu (%.1f%%)\n", 
                  currentData.frames_received, 
                  currentData.frames_error,
                  currentData.frames_received > 0 ? 
                  (100.0f * currentData.frames_error / currentData.frames_received) : 0);
    Serial.printf("Update rate: %.1f Hz\n", currentData.update_rate_hz);
    Serial.printf("Stato: %s\n", currentData.is_static ? "STATICO" : "MOVIMENTO");
    
    if (currentData.drift_compensation_enabled) {
        Serial.printf("Drift yaw: %.3f°/min\n", currentData.yaw_drift_rate);
    }
}

// === STATISTICHE ===
void updateStatistics() {
    static uint32_t last_stat_time = 0;
    static uint32_t last_frame_count = 0;
    
    uint32_t now = millis();
    if (now - last_stat_time > 1000) {
        uint32_t frames_delta = currentData.frames_received - last_frame_count;
        currentData.update_rate_hz = frames_delta;
        
        last_frame_count = currentData.frames_received;
        last_stat_time = now;
    }
}

// === EEPROM sostituita da Preferences ===

// === SOSTITUISCI LE FUNZIONI EEPROM ===

bool saveHWT906Calibration() {
    preferences.begin("hwt906", false);  // false = read/write mode
    
    preferences.putFloat("roll_off", currentData.roll_offset);
    preferences.putFloat("pitch_off", currentData.pitch_offset);
    preferences.putFloat("yaw_off", currentData.yaw_offset);
    preferences.putBool("calibrated", true);
    
    preferences.end();
    
    Serial.println("✅ Calibrazione HWT906 salvata");
    return true;
}

bool loadHWT906Calibration() {
    preferences.begin("hwt906", true);  // true = read-only mode
    
    // Verifica se esiste calibrazione
    if (!preferences.getBool("calibrated", false)) {
        preferences.end();
        return false;
    }
    
    currentData.roll_offset = preferences.getFloat("roll_off", 0.0);
    currentData.pitch_offset = preferences.getFloat("pitch_off", 0.0);
    currentData.yaw_offset = preferences.getFloat("yaw_off", 0.0);
    
    preferences.end();
    
    Serial.printf("✅ Calibrazione caricata: R=%.1f P=%.1f Y=%.1f\n",
                  currentData.roll_offset, 
                  currentData.pitch_offset, 
                  currentData.yaw_offset);
    return true;
}

// === CONFIGURAZIONE AVANZATA ===
bool configureHWT906Mode(HWT906PrecisionMode mode) {
    if (!currentData.sensor_present) return false;
    
    Serial.printf("⚙️ Configurazione modo: %d\n", mode);
    
    if (!unlockHWT906()) return false;
    
    switch (mode) {
        case HWT906_MODE_STATIC_HIGH:
            writeRegister(0x03, 0x0005);  // 5Hz
            writeRegister(0x1F, 0x0006);  // Max bandwidth filter
            break;
            
        case HWT906_MODE_STATIC_MED:
            writeRegister(0x03, 0x0006);  // 10Hz
            writeRegister(0x1F, 0x0005);  // Medium filter
            break;
            
        case HWT906_MODE_DYNAMIC:
            writeRegister(0x03, 0x0008);  // 50Hz
            writeRegister(0x1F, 0x0003);  // Light filter
            break;
            
        case HWT906_MODE_RAW:
            writeRegister(0x03, 0x0009);  // 100Hz
            writeRegister(0x1F, 0x0001);  // Minimal filter
            break;
    }
    
    writeRegister(0x00, 0x0000);  // Save
    delay(1000);
    
    return true;
}

void enableHWT906DriftCompensation(bool enable) {
    currentData.drift_compensation_enabled = enable;
    Serial.printf("📊 Compensazione drift: %s\n", enable ? "ATTIVA" : "DISATTIVA");
}

float getHWT906UpdateRate() {
    return currentData.update_rate_hz;
}

void resetHWT906() {
    // Reset software
    buffer_index = 0;
    first_reading = true;
    currentData.valid = false;
    currentData.frames_received = 0;
    currentData.frames_error = 0;
    Serial.println("🔄 HWT906 reset software");
}

void restartHWT906() {
    // Restart hardware
    if (currentData.sensor_present) {
        unlockHWT906();
        writeRegister(0x00, 0x00FF);  // Restart command
        delay(2000);
        initHWT906();
    }
}

bool testHWT906Communication() {
    return currentData.sensor_present && 
           (millis() - currentData.last_frame_time < TIMEOUT_MS);
}

void printHWT906Config() {
    Serial.println("\n=== HWT906 CONFIG ===");
    Serial.printf("I2C Address: 0x%02X\n", HWT906_I2C_ADDR);
    Serial.printf("UART: RX=%d TX=%d @ %d baud\n", 
                  HWT906_RX_PIN, HWT906_TX_PIN, HWT906_BAUD);
    Serial.printf("Filtri: EMA=%.2f, DeadZone P=%.2f° Y=%.2f°\n",
                  HWT906_EMA_ALPHA, HWT906_PITCH_DEADZONE, HWT906_YAW_DEADZONE);
    Serial.printf("Calibrato: %s\n", currentData.is_calibrated ? "SI" : "NO");
}
