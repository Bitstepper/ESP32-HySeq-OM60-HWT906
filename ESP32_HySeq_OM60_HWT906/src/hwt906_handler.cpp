// hwt906_handler.cpp - Implementazione completa basata su WIT9-proto funzionante
// Include TUTTI i filtri, drift compensation e parsing robusto
// VERSIONE CORRETTA - Allineata agli errori di compilazione
// STEP X1: AGGIUNTA SALVATAGGIO VALORI RAW per test ripetibilità 
// STEP X2: DUAL-MODE QUATERNIONS + EULER per anti-gimbal lock

#include "hwt906_handler.h"

// === ISTANZA GLOBALE ===
HWT906Handler hwt906;

// === PIN DEFINITIONS (mancanti) ===
#define HWT906_RX_PIN 44
#define HWT906_TX_PIN 43

// === FRAME DEFINITIONS AGGIUNTIVE ===
#define FRAME_QUATERNION 0x59

// === IMPLEMENTAZIONE CLASSE PRINCIPALE ===

bool HWT906Handler::init(HardwareSerial* serial_port) {
    Serial.println("\n=== HWT906 INIT - WIT9-proto Enhanced + Dual-Mode ===");
    
    uart = serial_port;
    
    // UART init 
    uart->begin(HWT906_UART_SPEED, SERIAL_8N1, HWT906_RX_PIN, HWT906_TX_PIN); // 9600, 44, 43
    uart->setTimeout(10);
    
    // === STEP X1: INIT RAW DATA ===
    rawData.pitch_raw_uart = 0.0f;
    rawData.yaw_raw_uart = 0.0f;
    rawData.roll_raw_uart = 0.0f;
    rawData.timestamp_raw = 0;
    rawData.valid = false;
    
    // === STEP X2: INIT QUATERNION DATA ===
    currentData.qw_raw = 1.0f;  // Identity quaternion
    currentData.qx_raw = 0.0f;
    currentData.qy_raw = 0.0f;
    currentData.qz_raw = 0.0f;
    currentData.qw_filtered = 1.0f;
    currentData.qx_filtered = 0.0f;
    currentData.qy_filtered = 0.0f;
    currentData.qz_filtered = 0.0f;
    currentData.quaternion_available = false;
    currentData.gimbal_lock_detected = false;
    currentData.quaternion_mode_active = false;
    currentData.last_quaternion_time = 0;
    currentData.quaternion_frames_received = 0;
    currentData.euler_frames_received = 0;
    
    // Init strutture filtri
    initAdvancedFilters();
    initDriftCompensation();
    
    // Carica calibrazione da EEPROM
    loadCalibration();
    
    // I2C init per configurazione
    Wire.begin();
    
    // Configura modalità statica ottimizzata con dual-mode
    bool config_ok = configureStaticOptimized(current_mode);
    
    if (config_ok) {
        Serial.printf("✅ HWT906 Configured: %s (DUAL-MODE)\n", precision_configs[current_mode].description);
        
        // Test di comunicazione UART
        delay(500);
        uint32_t test_start = millis();
        int valid_frames = 0;
        
        while (millis() - test_start < 2000) {
            if (parseUARTData()) {
                valid_frames++;
                if (valid_frames >= 3) break;
            }
            delay(10);
        }
        
        sensors_ready = (valid_frames >= 3);
        
        if (sensors_ready) {
            Serial.printf("✅ HWT906 Ready - %d valid frames (Dual-Mode enabled)\n", valid_frames);
            currentData.last_frame_time = millis();
        } else {
            Serial.println("❌ HWT906 UART Communication Failed");
        }
    } else {
        Serial.println("❌ HWT906 Configuration Failed");
    }
    
    return sensors_ready;
}

bool HWT906Handler::configureStaticOptimized(HWT906PrecisionMode mode) {
    Serial.printf("\n=== STATIC CONFIG DUAL-MODE: %s ===\n", precision_configs[mode].description);
    
    // Unlock per scrittura registri
    if (!unlockHWT906()) {
        Serial.println("❌ Unlock failed");
        return false;
    }
    
    // === STEP X2: ENABLE DUAL OUTPUT (ANGLE + QUATERNIONS) ===
    Serial.println("🔄 Enabling dual-mode output: Euler + Quaternions...");
    if (!writeRegister(0x02, 0x021E)) {  // ANGLE(0x53) + QUATER(0x59) + ACC + GYRO + MAG
        Serial.println("❌ Dual mode config failed");
        return false;
    }
    delay(200);  // Extra delay for dual-mode
    
    // CRITICO: Range ridotti per massima risoluzione (dal WIT9-proto)
    Serial.println("Setting reduced ranges for max resolution...");
    if (!writeRegister(0x21, 0x0000)) {  // ±2g (8X risoluzione!)
        Serial.println("❌ Accelerometer range failed");
        return false;
    }
    
    if (!writeRegister(0x20, 0x0000)) {  // ±250°/s 
        Serial.println("❌ Gyroscope range failed"); 
        return false;
    }
    
    // Output rate ottimizzato
    if (!writeRegister(0x03, precision_configs[mode].output_rate)) {
        Serial.println("❌ Output rate failed");
        return false;
    }
    
    // Algoritmo 9-assi per stabilità 
    if (!writeRegister(0x24, 0x0000)) {  // 9-axis
        Serial.println("❌ Algorithm mode failed");
        return false;
    }
    
    // Filtri ottimizzati
    writeRegister(0x1F, precision_configs[mode].bandwidth);
    writeRegister(0x25, precision_configs[mode].kalman_k);
    writeRegister(0x2A, precision_configs[mode].acc_filter);
    
    // Soglie statiche
    writeRegister(0x61, precision_configs[mode].gyro_threshold);
    writeRegister(0x63, precision_configs[mode].gyro_cal_time);
    
    // Salva configurazione
    if (!writeRegister(0x00, 0x0000)) {
        Serial.println("❌ Save config failed");
        return false;
    }
    
    // Lock
    lockHWT906();
    
    delay(500); // Stabilizzazione
    current_mode = mode;
    
    Serial.println("✅ Dual-mode static configuration completed");
    return true;
}

void HWT906Handler::update() {
    // Parse dati UART disponibili
    while (parseUARTData()) {
        // Processo frame ricevuti
    }
    
    // Applica pipeline filtri multi-stadio (CHIAVE del WIT9-proto)
    applyAdvancedFiltering();
    
    // Compensazione drift
    compensateDrift();
    
    // Rilevamento stato statico
    detectStaticState();
    
    // Aggiorna statistiche
    updateStatistics();
    
    // Debug controllato (ogni 50 frame)
    if (debug_counter++ % DEBUG_INTERVAL == 0) {
        printDebugInfo();
    }
}

bool HWT906Handler::parseUARTData() {
    if (!uart || !uart->available()) return false;
    
    while (uart->available()) {
        uint8_t byte = uart->read();
        
        if (!frame_started) {
            if (byte == FRAME_HEADER) {
                frame_started = true;
                frame_index = 0;
                frame_buffer[frame_index++] = byte;
            }
        } else {
            frame_buffer[frame_index++] = byte;
            
            if (frame_index >= FRAME_SIZE) {
                // Frame completo - processa
                if (validateFrame(frame_buffer)) {
                    if (parseFrame(frame_buffer)) {
                        currentData.frame_count++;
                        currentData.last_frame_time = millis();
                        frame_started = false;
                        return true; // Frame valido processato
                    }
                } else {
                    currentData.error_frames++;
                }
                
                frame_started = false;
                frame_index = 0;
            }
        }
    }
    
    return false;
}

bool HWT906Handler::validateFrame(uint8_t* frame) {
    // Verifica header
    if (frame[0] != FRAME_HEADER) return false;
    
    // Verifica checksum
    uint8_t sum = 0;
    for (int i = 0; i < FRAME_SIZE - 1; i++) {
        sum += frame[i];
    }
    
    return (sum == frame[FRAME_SIZE - 1]);
}

bool HWT906Handler::parseFrame(uint8_t* frame) {
    uint8_t frame_type = frame[1];
    
    switch (frame_type) {
        case FRAME_ANGLE:
            processAngleFrame(frame);
            currentData.euler_frames_received++;
            return true;
        
        // === STEP X2: NUOVO CASE PER QUATERNIONI ===
        case FRAME_QUATERNION:  // 0x59
            processQuaternionFrame(frame);
            currentData.quaternion_frames_received++;
            return true;
            
        case FRAME_ACCEL:
            processAccelFrame(frame);
            return true;
            
        case FRAME_GYRO:
            processGyroFrame(frame);
            return true;
            
        case FRAME_MAG:
            processMagFrame(frame);
            return true;
            
        default:
            return false;
    }
}

void HWT906Handler::processAngleFrame(uint8_t* frame) {
    // PARSING CORRETTO (dal WIT9-proto) - Low byte FIRST
    int16_t roll_raw = (int16_t)(frame[2] | (frame[3] << 8));
    int16_t pitch_raw = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t yaw_raw = (int16_t)(frame[6] | (frame[7] << 8));
    
    // Conversione in gradi (32768 = 180°)
    currentData.roll_raw = ((float)roll_raw / 32768.0f) * 180.0f;
    currentData.pitch_raw = ((float)pitch_raw / 32768.0f) * 90.0f;
    currentData.yaw_raw = ((float)yaw_raw / 32768.0f) * 180.0f;
    
    // === STEP X1: SALVA VALORI RAW PRIMA DI QUALSIASI PROCESSING ===
    // Questo è il momento CRITICO per catturare i dati puri dal sensore
    rawData.roll_raw_uart = currentData.roll_raw;
    rawData.pitch_raw_uart = currentData.pitch_raw;
    rawData.yaw_raw_uart = currentData.yaw_raw;
    rawData.timestamp_raw = millis();
    rawData.valid = true;
    
    // Normalizza YAW a [-180, +180] (anche per raw)
    while (rawData.yaw_raw_uart > 180.0f) rawData.yaw_raw_uart -= 360.0f;
    while (rawData.yaw_raw_uart < -180.0f) rawData.yaw_raw_uart += 360.0f;
    
    // Normalizza YAW a [-180, +180]
    while (currentData.yaw_raw > 180.0f) currentData.yaw_raw -= 360.0f;
    while (currentData.yaw_raw < -180.0f) currentData.yaw_raw += 360.0f;
    
    // Applica offset calibrazione
    currentData.roll = currentData.roll_raw - currentData.roll_offset;
    currentData.pitch = currentData.pitch_raw - currentData.pitch_offset;
    currentData.yaw = currentData.yaw_raw - currentData.yaw_offset;
    
    // Aggiorna range monitoring
    if (currentData.pitch < currentData.pitch_min) currentData.pitch_min = currentData.pitch;
    if (currentData.pitch > currentData.pitch_max) currentData.pitch_max = currentData.pitch;
    if (currentData.yaw < currentData.yaw_min) currentData.yaw_min = currentData.yaw;
    if (currentData.yaw > currentData.yaw_max) currentData.yaw_max = currentData.yaw;
}

// === STEP X2: NUOVA FUNZIONE PROCESSAMENTO QUATERNIONI ===
void HWT906Handler::processQuaternionFrame(uint8_t* frame) {
    // Parse quaternioni raw (Witmotion protocol: w,x,y,z)
    int16_t q0_raw = (int16_t)(frame[2] | (frame[3] << 8));  // w
    int16_t q1_raw = (int16_t)(frame[4] | (frame[5] << 8));  // x
    int16_t q2_raw = (int16_t)(frame[6] | (frame[7] << 8));  // y
    int16_t q3_raw = (int16_t)(frame[8] | (frame[9] << 8));  // z
    
    // Conversione a float normalizzato (-1.0 to +1.0)
    currentData.qw_raw = q0_raw / 32768.0f;
    currentData.qx_raw = q1_raw / 32768.0f;
    currentData.qy_raw = q2_raw / 32768.0f;
    currentData.qz_raw = q3_raw / 32768.0f;
    
    // Normalizzazione quaternione per sicurezza
    normalizeQuaternion(currentData.qw_raw, currentData.qx_raw, currentData.qy_raw, currentData.qz_raw);
    
    // Applica filtri ai quaternioni
    applyQuaternionFilters();
    
    // Converti a Eulero (sia raw che filtered)
    convertQuaternionToEuler();
    convertQuaternionToEulerFiltered();
    
    // Detect gimbal lock
    detectGimbalLock();
    
    // Update status
    currentData.last_quaternion_time = millis();
    currentData.quaternion_mode_active = true;
    currentData.quaternion_available = true;
    
    // Debug quaternioni (ridotto)
    if (debug_counter % (DEBUG_INTERVAL * 5) == 0) {
        Serial.printf("Q: w=%.3f x=%.3f y=%.3f z=%.3f -> P_Q=%.1f Y_Q=%.1f\n",
                     currentData.qw_raw, currentData.qx_raw, currentData.qy_raw, currentData.qz_raw,
                     currentData.pitch_Q, currentData.yaw_Q);
    }
}

void HWT906Handler::processAccelFrame(uint8_t* frame) {
    int16_t ax_raw = (int16_t)(frame[2] | (frame[3] << 8));
    int16_t ay_raw = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t az_raw = (int16_t)(frame[6] | (frame[7] << 8));
    
    // Conversione in g (32768 = 16g)
    currentData.ax = ((float)ax_raw / 32768.0f) * 16.0f;
    currentData.ay = ((float)ay_raw / 32768.0f) * 16.0f;
    currentData.az = ((float)az_raw / 32768.0f) * 16.0f;
}

void HWT906Handler::processGyroFrame(uint8_t* frame) {
    int16_t gx_raw = (int16_t)(frame[2] | (frame[3] << 8));
    int16_t gy_raw = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t gz_raw = (int16_t)(frame[6] | (frame[7] << 8));
    
    // Conversione in °/s (32768 = 2000°/s)
    currentData.gx = ((float)gx_raw / 32768.0f) * 2000.0f;
    currentData.gy = ((float)gy_raw / 32768.0f) * 2000.0f;
    currentData.gz = ((float)gz_raw / 32768.0f) * 2000.0f;
}

void HWT906Handler::processMagFrame(uint8_t* frame) {
    int16_t mx_raw = (int16_t)(frame[2] | (frame[3] << 8));
    int16_t my_raw = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t mz_raw = (int16_t)(frame[6] | (frame[7] << 8));
    
    // Normalizzazione magnetometro
    currentData.mx = (float)mx_raw;
    currentData.my = (float)my_raw;
    currentData.mz = (float)mz_raw;
}

// === STEP X2: NUOVE FUNZIONI QUATERNIONI ===

void HWT906Handler::normalizeQuaternion(float& qw, float& qx, float& qy, float& qz) {
    float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.001f) {  // Evita divisione per zero
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    } else {
        // Quaternione identità se norm troppo piccola
        qw = 1.0f;
        qx = qy = qz = 0.0f;
    }
}

void HWT906Handler::applyQuaternionFilters() {
    // Filtro EMA per quaternioni (mantiene normalizzazione)
    static bool first_run = true;
    if (first_run) {
        currentData.qw_filtered = currentData.qw_raw;
        currentData.qx_filtered = currentData.qx_raw;
        currentData.qy_filtered = currentData.qy_raw;
        currentData.qz_filtered = currentData.qz_raw;
        first_run = false;
        return;
    }
    
    float alpha = 0.1f;  // Smoothing factor (stesso degli Eulero)
    
    // SLERP semplificato per tempo reale (EMA lineare + normalizzazione)
    currentData.qw_filtered = currentData.qw_filtered * (1-alpha) + currentData.qw_raw * alpha;
    currentData.qx_filtered = currentData.qx_filtered * (1-alpha) + currentData.qx_raw * alpha;
    currentData.qy_filtered = currentData.qy_filtered * (1-alpha) + currentData.qy_raw * alpha;
    currentData.qz_filtered = currentData.qz_filtered * (1-alpha) + currentData.qz_raw * alpha;
    
    // Re-normalizzazione dopo filtro (CRITICO per quaternioni)
    normalizeQuaternion(currentData.qw_filtered, currentData.qx_filtered, currentData.qy_filtered, currentData.qz_filtered);
}

void HWT906Handler::convertQuaternionToEuler() {
    // Conversione quaternioni RAW -> Eulero
    float qw = currentData.qw_raw;
    float qx = currentData.qx_raw;
    float qy = currentData.qy_raw;
    float qz = currentData.qz_raw;
    
    // Roll (X-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    currentData.roll_Q = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
    
    // Pitch (Y-axis rotation) - SAFE VERSION contro gimbal lock
    float sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1) {
        currentData.pitch_Q = copysign(90.0f, sinp);  // ±90° limit
    } else {
        currentData.pitch_Q = asin(sinp) * 180.0f / PI;
    }
    
    // Yaw (Z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    currentData.yaw_Q = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
    
    // Normalizza YAW_Q a [-180, +180]
    while (currentData.yaw_Q > 180.0f) currentData.yaw_Q -= 360.0f;
    while (currentData.yaw_Q < -180.0f) currentData.yaw_Q += 360.0f;
}

void HWT906Handler::convertQuaternionToEulerFiltered() {
    // Conversione quaternioni FILTRATI -> Eulero
    float qw = currentData.qw_filtered;
    float qx = currentData.qx_filtered;
    float qy = currentData.qy_filtered;
    float qz = currentData.qz_filtered;
    
    // Stesso algoritmo ma sui quaternioni filtrati
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    currentData.roll_Q_filtered = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
    
    float sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1) {
        currentData.pitch_Q_filtered = copysign(90.0f, sinp);
    } else {
        currentData.pitch_Q_filtered = asin(sinp) * 180.0f / PI;
    }
    
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    currentData.yaw_Q_filtered = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
    
    // Normalizza YAW_Q_filtered a [-180, +180]
    while (currentData.yaw_Q_filtered > 180.0f) currentData.yaw_Q_filtered -= 360.0f;
    while (currentData.yaw_Q_filtered < -180.0f) currentData.yaw_Q_filtered += 360.0f;
}

void HWT906Handler::detectGimbalLock() {
    // Detect gimbal lock quando pitch si avvicina a ±90°
    bool euler_gimbal = (abs(currentData.pitch_filtered) > 85.0f);
    bool quaternion_gimbal = (abs(currentData.pitch_Q_filtered) > 85.0f);
    
    currentData.gimbal_lock_detected = (euler_gimbal || quaternion_gimbal);
    
    // Log quando rileva gimbal lock
    static bool last_gimbal_state = false;
    if (currentData.gimbal_lock_detected != last_gimbal_state) {
        if (currentData.gimbal_lock_detected) {
            Serial.printf("⚠️ GIMBAL LOCK detected: Pitch=%.1f° Pitch_Q=%.1f°\n", 
                         currentData.pitch_filtered, currentData.pitch_Q_filtered);
        } else {
            Serial.println("✅ GIMBAL LOCK cleared");
        }
        last_gimbal_state = currentData.gimbal_lock_detected;
    }
}

// === FILTRI MULTI-STADIO (CUORE del WIT9-proto) ===

void HWT906Handler::initAdvancedFilters() {
    // Reset buffer
    memset(filters.roll_buffer, 0, sizeof(filters.roll_buffer));
    memset(filters.pitch_buffer, 0, sizeof(filters.pitch_buffer));
    memset(filters.yaw_buffer, 0, sizeof(filters.yaw_buffer));
    filters.buffer_index = 0;
    filters.buffer_full = false;
    
    // Init matrice covarianza Kalman
    for (int i = 0; i < 3; i++) {
        filters.P[i][0][0] = 1.0f;
        filters.P[i][0][1] = 0.0f;
        filters.P[i][1][0] = 0.0f;
        filters.P[i][1][1] = 1.0f;
    }
    
    Serial.println("✅ Advanced filters initialized (Enhanced for Dual-Mode)");
}

void HWT906Handler::applyAdvancedFiltering() {
    // PIPELINE MULTI-STADIO (strategia WIT9-proto):
    // 1. Media mobile per ridurre noise
    updateMovingAverage();
    
    // 2. Filtro complementare per pitch/roll
//    applyComplementaryFilter();
    
    // 3. Kalman per YAW (componente critica)
    applyKalmanFilterYaw();
    
    // 4. EMA finale per smoothing
    applyEMAFilter();
    
    // 5. Zone morte adattive per stabilità 
    applyStaticThreshold();
}

void HWT906Handler::updateMovingAverage() {
    // Buffer circolare
    filters.roll_buffer[filters.buffer_index] = currentData.roll;
    filters.pitch_buffer[filters.buffer_index] = currentData.pitch;
    filters.yaw_buffer[filters.buffer_index] = currentData.yaw;
    
    filters.buffer_index = (filters.buffer_index + 1) % 10;
    if (filters.buffer_index == 0) filters.buffer_full = true;
    
    // Calcola media mobile
    if (filters.buffer_full) {
        float roll_sum = 0, pitch_sum = 0, yaw_sum = 0;
        for (int i = 0; i < 10; i++) {
            roll_sum += filters.roll_buffer[i];
            pitch_sum += filters.pitch_buffer[i];
            yaw_sum += filters.yaw_buffer[i];
        }
        
        currentData.roll = roll_sum / 10.0f;
        currentData.pitch = pitch_sum / 10.0f;
        currentData.yaw = yaw_sum / 10.0f;
    }
}

void HWT906Handler::applyComplementaryFilter() {
    // Filtro complementare per pitch/roll usando accelerometro
    float acc_magnitude = sqrt(currentData.ax * currentData.ax + 
                              currentData.ay * currentData.ay + 
                              currentData.az * currentData.az);
    
    if (acc_magnitude > 0.5f && acc_magnitude < 1.5f) { // Range valido
        // Calcola angoli da accelerometro
        float acc_roll = atan2(currentData.ay, currentData.az) * 180.0f / PI;
        float acc_pitch = atan2(-currentData.ax, sqrt(currentData.ay * currentData.ay + currentData.az * currentData.az)) * 180.0f / PI;
        
        // Applica filtro complementare
        currentData.roll = filters.comp_alpha * currentData.roll + (1.0f - filters.comp_alpha) * acc_roll;
        currentData.pitch = filters.comp_alpha * currentData.pitch + (1.0f - filters.comp_alpha) * acc_pitch;
    }
}

void HWT906Handler::applyKalmanFilterYaw() {
    // Kalman semplificato solo per YAW (componente più critica)
    int idx = 2; // Yaw index
    
    // Predict
    filters.angle[idx] += (currentData.gz - filters.bias[idx]) * 0.01f; // dt = 10ms
    filters.P[idx][0][0] += 0.01f * (0.01f * filters.P[idx][1][1] - filters.P[idx][0][1] - filters.P[idx][1][0] + filters.Q_angle);
    filters.P[idx][0][1] -= 0.01f * filters.P[idx][1][1];
    filters.P[idx][1][0] -= 0.01f * filters.P[idx][1][1];
    filters.P[idx][1][1] += filters.Q_bias * 0.01f;
    
    // Update
    float innovation = currentData.yaw - filters.angle[idx];
    float S = filters.P[idx][0][0] + filters.R_measure;
    float K[2];
    K[0] = filters.P[idx][0][0] / S;
    K[1] = filters.P[idx][1][0] / S;
    
    filters.angle[idx] += K[0] * innovation;
    filters.bias[idx] += K[1] * innovation;
    
    float P00_temp = filters.P[idx][0][0];
    float P01_temp = filters.P[idx][0][1];
    
    filters.P[idx][0][0] -= K[0] * P00_temp;
    filters.P[idx][0][1] -= K[0] * P01_temp;
    filters.P[idx][1][0] -= K[1] * P00_temp;
    filters.P[idx][1][1] -= K[1] * P01_temp;
    
    currentData.yaw = filters.angle[idx];
}

void HWT906Handler::applyEMAFilter() {
    // EMA finale per smoothing
    currentData.roll_filtered = filters.ema_alpha * currentData.roll + (1.0f - filters.ema_alpha) * filters.last_roll;
    currentData.pitch_filtered = filters.ema_alpha * currentData.pitch + (1.0f - filters.ema_alpha) * filters.last_pitch;
    currentData.yaw_filtered = filters.ema_alpha * currentData.yaw + (1.0f - filters.ema_alpha) * filters.last_yaw;
    
    filters.last_roll = currentData.roll_filtered;
    filters.last_pitch = currentData.pitch_filtered;
    filters.last_yaw = currentData.yaw_filtered;
}

void HWT906Handler::applyStaticThreshold() {
    // Zone morte adattive per stabilità 
    uint32_t now = millis();
    
    // Pitch
    float pitch_delta = abs(currentData.pitch_filtered - filters.last_pitch);
    if (pitch_delta < filters.pitch_dead_zone) {
        currentData.pitch_filtered = filters.last_pitch;
    } else {
        filters.last_significant_change = now;
    }
    
    // Yaw con zona morta più ampia
    float yaw_delta = abs(currentData.yaw_filtered - filters.last_yaw);
    if (yaw_delta < filters.yaw_dead_zone) {
        currentData.yaw_filtered = filters.last_yaw;
    } else {
        filters.last_significant_change = now;
    }
}

// === DRIFT COMPENSATION (funzionalità chiave WIT9-proto) ===

void HWT906Handler::initDriftCompensation() {
    drift.enabled = true;
    drift.reference_time = millis();
    drift.yaw_reference = 0.0f;
    drift.history_index = 0;
    drift.history_full = false;
    
    memset(drift.yaw_history, 0, sizeof(drift.yaw_history));
    memset(drift.history_timestamps, 0, sizeof(drift.history_timestamps));
    
    Serial.println("✅ Drift compensation initialized");
}

void HWT906Handler::compensateDrift() {
    if (!drift.enabled) return;
    
    uint32_t now = millis();
    
    // Aggiorna storia ogni secondo
    if (now - drift.last_compensation >= 1000) {
        updateDriftHistory();
        
        // Calcola drift rate se abbiamo abbastanza storia
        if (drift.history_full || drift.history_index > 60) { // Almeno 1 minuto
            calculateDriftRate();
            
            // Applica compensazione
            if (abs(drift.yaw_drift_rate) < drift.MAX_COMP_RATE) {
                float compensation = drift.yaw_drift_rate * (now - drift.reference_time) / 1000.0f;
                currentData.yaw_filtered -= compensation;
            }
        }
        
        drift.last_compensation = now;
    }
}

void HWT906Handler::updateDriftHistory() {
    drift.yaw_history[drift.history_index] = currentData.yaw_filtered;
    drift.history_timestamps[drift.history_index] = millis();
    
    drift.history_index = (drift.history_index + 1) % 300;
    if (drift.history_index == 0) drift.history_full = true;
}

void HWT906Handler::calculateDriftRate() {
    // Regressione lineare per calcolare drift rate
    int samples = drift.history_full ? 300 : drift.history_index;
    if (samples < 30) return;
    
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    uint32_t base_time = drift.history_timestamps[0];
    
    for (int i = 0; i < samples; i++) {
        float x = (drift.history_timestamps[i] - base_time) / 1000.0f; // Secondi
        float y = drift.yaw_history[i];
        
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    
    float slope = (samples * sum_xy - sum_x * sum_y) / (samples * sum_x2 - sum_x * sum_x);
    drift.yaw_drift_rate = slope;
}

// === RILEVAMENTO STATO STATICO ===

void HWT906Handler::detectStaticState() {
    bool currently_static = (abs(currentData.gx) < currentData.static_threshold &&
                           abs(currentData.gy) < currentData.static_threshold &&
                           abs(currentData.gz) < currentData.static_threshold);
    
    if (currently_static != currentData.is_static) {
        if (currently_static) {
            currentData.is_static = true;
            currentData.static_start_time = millis();
            
            // Ottimizza filtri per stato statico
            filters.ema_alpha = 0.05f; // Più smoothing
            filters.comp_alpha = 0.99f; // Più peso al gyro
            
            Serial.println("🔒 Static state detected - enhanced filtering enabled");
        } else {
            uint32_t static_duration = millis() - currentData.static_start_time;
            Serial.printf("🔄 Dynamic state detected after %lus static\n", static_duration/1000);
            
            // Ripristina filtri dinamici
            filters.ema_alpha = 0.1f;
            filters.comp_alpha = 0.98f;
            
            currentData.is_static = false;
        }
    }
}

// === METODI I2C ===

bool HWT906Handler::unlockHWT906() {
    Wire.beginTransmission(HWT906_I2C_ADDR);
    Wire.write(0x69);
    Wire.write(0x88);
    Wire.write(0xB5);
    return (Wire.endTransmission() == 0);
}

bool HWT906Handler::writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(HWT906_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value & 0xFF);        // Low byte
    Wire.write((value >> 8) & 0xFF); // High byte
    return (Wire.endTransmission() == 0);
}

bool HWT906Handler::lockHWT906() {
    delay(10);
    return true; // Auto-lock dopo salvataggio
}

// === CALIBRAZIONE ===

void HWT906Handler::startCalibration() {
    Serial.println("🎯 Starting calibration...");
    
    // Reset offset
    currentData.pitch_offset = 0.0f;
    currentData.yaw_offset = 0.0f;
    currentData.roll_offset = 0.0f;
    
    // Raccoli dati per 3 secondi
    float pitch_sum = 0.0f, yaw_sum = 0.0f, roll_sum = 0.0f;
    int samples = 0;
    
    uint32_t start = millis();
    while (millis() - start < 3000) {
        if (parseUARTData()) {
            pitch_sum += currentData.pitch_raw;
            yaw_sum += currentData.yaw_raw;
            roll_sum += currentData.roll_raw;
            samples++;
        }
        delay(10);
    }
    
    if (samples > 0) {
        currentData.pitch_offset = pitch_sum / samples;
        currentData.yaw_offset = yaw_sum / samples;
        currentData.roll_offset = roll_sum / samples;
        
        saveCalibration();
        currentData.is_calibrated = true;
        
        Serial.printf("✅ Calibration complete: P=%.2f Y=%.2f R=%.2f\n", 
                     currentData.pitch_offset, currentData.yaw_offset, currentData.roll_offset);
    }
}

void HWT906Handler::saveCalibration() {
    EEPROM.put(HWT906_EEPROM_BASE, currentData.pitch_offset);
    EEPROM.put(HWT906_EEPROM_BASE + 4, currentData.yaw_offset);
    EEPROM.put(HWT906_EEPROM_BASE + 8, currentData.roll_offset);
    EEPROM.put(HWT906_EEPROM_BASE + 12, true); // Calibrated flag
    EEPROM.commit();
}

void HWT906Handler::loadCalibration() {
    bool calibrated;
    EEPROM.get(HWT906_EEPROM_BASE + 12, calibrated);
    
    if (calibrated) {
        EEPROM.get(HWT906_EEPROM_BASE, currentData.pitch_offset);
        EEPROM.get(HWT906_EEPROM_BASE + 4, currentData.yaw_offset);
        EEPROM.get(HWT906_EEPROM_BASE + 8, currentData.roll_offset);
        currentData.is_calibrated = true;
        
        Serial.printf("📂 Calibration loaded: P=%.2f Y=%.2f R=%.2f\n",
                     currentData.pitch_offset, currentData.yaw_offset, currentData.roll_offset);
    }
}

// === STATISTICHE E DEBUG ===

void HWT906Handler::updateStatistics() {
    uint32_t now = millis();
    static uint32_t last_update = 0;
    
    if (now - last_update >= 1000) { // Ogni secondo
        currentData.update_rate = (float)currentData.frame_count / ((now - last_update) / 1000.0f);
        last_update = now;
    }
}

void HWT906Handler::printDebugInfo() {
    if (debug_counter % (DEBUG_INTERVAL * 10) == 0) { // Ogni 500 frame
        Serial.printf("📊 HWT906 DUAL: P=%.2f° Y=%.2f° D=%.2f° | P_Q=%.2f° Y_Q=%.2f° | Rate=%.1fHz | GL=%s\n",
                     currentData.pitch_filtered, currentData.yaw_filtered, currentData.roll_filtered,
                     currentData.pitch_Q_filtered, currentData.yaw_Q_filtered,
                     currentData.update_rate, currentData.gimbal_lock_detected ? "YES" : "NO");
        
        Serial.printf("   Frames: E=%lu Q=%lu Err=%lu | %s\n",
                     currentData.euler_frames_received, currentData.quaternion_frames_received,
                     currentData.error_frames, currentData.is_static ? "STATIC" : "DYNAMIC");
    }
}

// === IMPLEMENTAZIONI METODI PUBBLICI MANCANTI ===

void HWT906Handler::setPrecisionMode(HWT906PrecisionMode mode) {
    current_mode = mode;
    configureStaticOptimized(mode);
}

void HWT906Handler::setDriftCompensation(bool enable) {
    drift.enabled = enable;
    Serial.printf("🔄 Drift compensation: %s\n", enable ? "ENABLED" : "DISABLED");
}

void HWT906Handler::saveCurrentAsZero() {
    currentData.pitch_offset = currentData.pitch_raw;
    currentData.yaw_offset = currentData.yaw_raw;
    currentData.roll_offset = currentData.roll_raw;
    
    saveCalibration();
    currentData.is_calibrated = true;
    
    Serial.printf("✅ Zero saved: P=%.2f° Y=%.2f° R=%.2f°\n", 
                 currentData.pitch_offset, currentData.yaw_offset, currentData.roll_offset);
}

void HWT906Handler::setDeadZones(float pitch_zone, float yaw_zone) {
    filters.pitch_dead_zone = pitch_zone;
    filters.yaw_dead_zone = yaw_zone;
    Serial.printf("🎯 Dead zones set: Pitch=%.2f° Yaw=%.2f°\n", pitch_zone, yaw_zone);
}

void HWT906Handler::getRange(float& pitch_min, float& pitch_max, float& yaw_min, float& yaw_max) {
    pitch_min = currentData.pitch_min;
    pitch_max = currentData.pitch_max;
    yaw_min = currentData.yaw_min;
    yaw_max = currentData.yaw_max;
}

void HWT906Handler::resetRange() {
    currentData.pitch_min = 999.0f;
    currentData.pitch_max = -999.0f;
    currentData.yaw_min = 999.0f;
    currentData.yaw_max = -999.0f;
    Serial.println("🔄 Range statistics reset");
}

void HWT906Handler::resetCalibration() {
    // Reset offset calibrazioni
    currentData.pitch_offset = 0.0f;
    currentData.yaw_offset = 0.0f;
    currentData.roll_offset = 0.0f;
    currentData.is_calibrated = false;
    
    // Cancella da EEPROM
    EEPROM.put(HWT906_EEPROM_BASE, 0.0f);        // pitch_offset
    EEPROM.put(HWT906_EEPROM_BASE + 4, 0.0f);    // yaw_offset  
    EEPROM.put(HWT906_EEPROM_BASE + 8, 0.0f);    // roll_offset
    EEPROM.put(HWT906_EEPROM_BASE + 12, false);  // calibrated flag
    EEPROM.commit();
    
    Serial.println("🔄 HWT906 calibration reset");
}

void HWT906Handler::printStatus() {
    Serial.println("\n=== HWT906 DUAL-MODE STATUS ===");
    Serial.printf("Ready: %s\n", sensors_ready ? "YES" : "NO");
    Serial.printf("Mode: %s\n", precision_configs[current_mode].description);
    Serial.printf("Dual-Mode: %s\n", currentData.quaternion_mode_active ? "ACTIVE" : "INACTIVE");
    Serial.printf("Gimbal Lock: %s\n", currentData.gimbal_lock_detected ? "DETECTED" : "OK");
    Serial.printf("Drift Comp: %s\n", drift.enabled ? "ON" : "OFF");
    Serial.printf("Static: %s\n", currentData.is_static ? "YES" : "NO");
    Serial.printf("Calibrated: %s\n", currentData.is_calibrated ? "YES" : "NO");
    Serial.printf("Update Rate: %.1f Hz\n", currentData.update_rate);
    Serial.printf("Frames: Euler=%lu Quat=%lu (Errors: %lu)\n", 
                 currentData.euler_frames_received, currentData.quaternion_frames_received, currentData.error_frames);
    Serial.println("==============================\n");
}

void HWT906Handler::enableDebug(bool enable) {
    // Toggle debug output frequency
    // Implementation can be expanded later
    Serial.printf("🔍 Debug mode: %s\n", enable ? "ON" : "OFF");
}

// === FUNZIONI MAGNETOMETRO STUB (per compatibilità OM60) ===

void startMagCalibration() {
    Serial.println("🧭 Magnetometer calibration not applicable for HWT906");
    Serial.println("   (HWT906 uses internal magnetometer calibration)");
}

bool isMagCalibrationInProgress() {
    return false; // HWT906 calibration is internal
}

float getMagCalibrationProgress() {
    return 100.0f; // Always "complete" for HWT906
}

void finishMagCalibration() {
    Serial.println("✅ Magnetometer calibration: N/A for HWT906");
}

// === API WRAPPER GLOBALI (compatibilità) ===

bool initHWT906() {
    return hwt906.init();
}

void updateHWT906() {
    hwt906.update();
}

float getHWT906Pitch() {
    return hwt906.getPitch();
}

float getHWT906Yaw() {
    return hwt906.getYaw();
}

float getHWT906Roll() {
    return hwt906.getRoll();
}

bool isHWT906Ready() {
    return hwt906.isReady();
}

bool isHWT906Present() {
    return hwt906.isReady();
}

void calibrateHWT906() {
    hwt906.startCalibration();
}

void zeroHWT906Angles() {
    hwt906.saveCurrentAsZero();
}

// === STEP X1: API WRAPPER RAW DATA ===
float getHWT906PitchRaw() {
    return hwt906.getPitchRawUART();
}

float getHWT906YawRaw() {
    return hwt906.getYawRawUART();
}

float getHWT906RollRaw() {
    return hwt906.getRollRawUART();
}

HWT906RawData getHWT906RawData() {
    return hwt906.getRawData();
}

// === STEP X2: API WRAPPER QUATERNIONS ===
float getHWT906QuaternionW() {
    return hwt906.getQuaternionW();
}

float getHWT906QuaternionX() {
    return hwt906.getQuaternionX();
}

float getHWT906QuaternionY() {
    return hwt906.getQuaternionY();
}

float getHWT906QuaternionZ() {
    return hwt906.getQuaternionZ();
}

float getHWT906PitchQ() {
    return hwt906.getPitchQ();
}

float getHWT906YawQ() {
    return hwt906.getYawQ();
}

float getHWT906RollQ() {
    return hwt906.getRollQ();
}

float getHWT906PitchQFiltered() {
    return hwt906.getPitchQFiltered();
}

float getHWT906YawQFiltered() {
    return hwt906.getYawQFiltered();
}

float getHWT906RollQFiltered() {
    return hwt906.getRollQFiltered();
}

bool isHWT906GimbalLockDetected() {
    return hwt906.isGimbalLockDetected();
}
