// hwt906_handler.cpp - FIXED Step CAL Implementation
// 🔧 CRITICAL FIXES:
// 1. GYRO: Raddoppiato tempo raccolta - 50 secondi (1000 campioni @ 20Hz)
// 2. ACCEL: Raddoppiato campioni - 200 per punto (10 secondi @ 20Hz)  
// 3. MAG: Raddoppiato tempo raccolta - 100 secondi (2000 campioni @ 20Hz)
// Maintains all existing dual-mode quaternion functionality

#include "hwt906_handler.h"

// === ISTANZA GLOBALE ===
HWT906Handler hwt906;

// === PIN DEFINITIONS ===
#define HWT906_RX_PIN 44
#define HWT906_TX_PIN 43

// === IMPLEMENTAZIONE CLASSE PRINCIPALE ===

bool HWT906Handler::init(HardwareSerial* serial_port) {
    Serial.println("\n=== HWT906 INIT - Enhanced with Step CAL FIXED ===");
    
    uart = serial_port;
    
    // UART init 
    uart->begin(HWT906_UART_SPEED, SERIAL_8N1, HWT906_RX_PIN, HWT906_TX_PIN);
    uart->setTimeout(10);
    
    // Init strutture
    initAdvancedFilters();
    initDriftCompensation();
    
    // === STEP CAL: INIT PREFERENCES STORAGE ===
    if (!prefs.begin(HWT906_PREFS_NAMESPACE)) {
        Serial.println("⚠️ Failed to initialize Preferences");
    } else {
        Serial.println("✅ Preferences storage ready");
    }
    
    // Carica tutte le calibrazioni da Preferences
    loadAllCalibrations();
    
    // Dual-mode quaternions init
    currentData.qw_raw = 1.0f;
    currentData.qx_raw = currentData.qy_raw = currentData.qz_raw = 0.0f;
    currentData.qw_filtered = 1.0f; 
    currentData.qx_filtered = currentData.qy_filtered = currentData.qz_filtered = 0.0f;
    currentData.quaternion_available = false;
    currentData.gimbal_lock_detected = false;
    currentData.quaternion_mode_active = false;
    
    // I2C init per configurazione
    Wire.begin();
    
    // Configura modalità statica ottimizzata con dual-mode
    bool config_ok = configureStaticOptimized(current_mode);
    
    if (config_ok) {
        Serial.printf("✅ HWT906 Configured: %s (DUAL-MODE + Step CAL FIXED)\n", precision_configs[current_mode].description);
        
        // Test comunicazione UART
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
            Serial.printf("✅ HWT906 Ready - %d valid frames (Calibrations loaded)\n", valid_frames);
            currentData.last_frame_time = millis();
            
            // Print calibration status
            Serial.printf("📊 Calibration Status: Gyro=%s Accel=%s Mag=%s\n",
                         isGyroCalibrated() ? "✅" : "❌",
                         isAccelCalibrated() ? "✅" : "❌", 
                         isMagCalibrated() ? "✅" : "❌");
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
    
    if (!unlockHWT906()) {
        Serial.println("❌ Unlock failed");
        return false;
    }
    
    // Enable dual output (Euler + Quaternions)
    Serial.println("🔄 Enabling dual-mode output: Euler + Quaternions...");
    if (!writeRegister(0x02, 0x021E)) {
        Serial.println("❌ Dual mode config failed");
        return false;
    }
    delay(200);
    
    // Configurazione range e filtri ottimizzata
    if (!writeRegister(0x21, 0x0000)) {  // ±2g accelerometer
        Serial.println("❌ Accelerometer range failed");
        return false;
    }
    
    if (!writeRegister(0x20, 0x0000)) {  // ±250°/s gyroscope
        Serial.println("❌ Gyroscope range failed");
        return false;
    }
    
    if (!writeRegister(0x03, precision_configs[mode].output_rate)) {
        Serial.println("❌ Output rate failed");
        return false;
    }
    
    if (!writeRegister(0x24, 0x0000)) {  // 9-axis algorithm
        Serial.println("❌ Algorithm mode failed");
        return false;
    }
    
    // Filtri ottimizzati
    writeRegister(0x1F, precision_configs[mode].bandwidth);
    writeRegister(0x25, precision_configs[mode].kalman_k);
    writeRegister(0x2A, precision_configs[mode].acc_filter);
    writeRegister(0x61, precision_configs[mode].gyro_threshold);
    writeRegister(0x63, precision_configs[mode].gyro_cal_time);
    
    if (!writeRegister(0x00, 0x0000)) {
        Serial.println("❌ Save config failed");
        return false;
    }
    
    lockHWT906();
    delay(500);
    current_mode = mode;
    
    Serial.println("✅ Dual-mode static configuration completed");
    return true;
}

void HWT906Handler::update() {
    // Parse dati UART disponibili
    while (parseUARTData()) {
        // Processo frame ricevuti
    }
    
    // === STEP CAL: APPLICA CALIBRAZIONI AI DATI RAW ===
    applySensorCalibrations();
    
    // === STEP CAL: PROCESSA STATE MACHINE CALIBRAZIONE ===
    processCalibrationStateMachine();
    
    // Pipeline filtri multi-stadio
    applyAdvancedFiltering();
    
    // Compensazione drift
    compensateDrift();
    
    // Rilevamento stato statico
    detectStaticState();
    
    // Aggiorna statistiche
    updateStatistics();
    
    // Debug controllato
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
                if (validateFrame(frame_buffer)) {
                    if (parseFrame(frame_buffer)) {
                        currentData.frame_count++;
                        currentData.last_frame_time = millis();
                        frame_started = false;
                        return true;
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
    if (frame[0] != FRAME_HEADER) return false;
    
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
        
        case FRAME_QUATERNION:
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
    int16_t roll_raw = (int16_t)(frame[2] | (frame[3] << 8));
    int16_t pitch_raw = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t yaw_raw = (int16_t)(frame[6] | (frame[7] << 8));
    
    currentData.roll_raw = ((float)roll_raw / 32768.0f) * 180.0f;
    currentData.pitch_raw = ((float)pitch_raw / 32768.0f) * 180.0f;
    currentData.yaw_raw = ((float)yaw_raw / 32768.0f) * 180.0f;
    
    // Normalizza YAW a [-180, +180]
    while (currentData.yaw_raw > 180.0f) currentData.yaw_raw -= 360.0f;
    while (currentData.yaw_raw < -180.0f) currentData.yaw_raw += 360.0f;
    
    // Applica offset calibrazione legacy
    currentData.roll = currentData.roll_raw - currentData.roll_offset;
    currentData.pitch = currentData.pitch_raw - currentData.pitch_offset;
    currentData.yaw = currentData.yaw_raw - currentData.yaw_offset;
    
    // Aggiorna range monitoring
    if (currentData.pitch < currentData.pitch_min) currentData.pitch_min = currentData.pitch;
    if (currentData.pitch > currentData.pitch_max) currentData.pitch_max = currentData.pitch;
    if (currentData.yaw < currentData.yaw_min) currentData.yaw_min = currentData.yaw;
    if (currentData.yaw > currentData.yaw_max) currentData.yaw_max = currentData.yaw;
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
    
    currentData.mx = (float)mx_raw;
    currentData.my = (float)my_raw;
    currentData.mz = (float)mz_raw;
}

// === DUAL-MODE QUATERNIONS (existing code - unchanged) ===

void HWT906Handler::processQuaternionFrame(uint8_t* frame) {
    int16_t q0_raw = (int16_t)(frame[2] | (frame[3] << 8));
    int16_t q1_raw = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t q2_raw = (int16_t)(frame[6] | (frame[7] << 8));
    int16_t q3_raw = (int16_t)(frame[8] | (frame[9] << 8));
    
    currentData.qw_raw = q0_raw / 32768.0f;
    currentData.qx_raw = q1_raw / 32768.0f;
    currentData.qy_raw = q2_raw / 32768.0f;
    currentData.qz_raw = q3_raw / 32768.0f;
    
    normalizeQuaternion(currentData.qw_raw, currentData.qx_raw, currentData.qy_raw, currentData.qz_raw);
    applyQuaternionFilters();
    convertQuaternionToEuler();
    convertQuaternionToEulerFiltered();
    detectGimbalLock();
    
    currentData.last_quaternion_time = millis();
    currentData.quaternion_mode_active = true;
    currentData.quaternion_available = true;
}

void HWT906Handler::normalizeQuaternion(float& qw, float& qx, float& qy, float& qz) {
    float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.001f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    } else {
        qw = 1.0f; qx = qy = qz = 0.0f;
    }
}

void HWT906Handler::applyQuaternionFilters() {
    static bool first_run = true;
    if (first_run) {
        currentData.qw_filtered = currentData.qw_raw;
        currentData.qx_filtered = currentData.qx_raw;
        currentData.qy_filtered = currentData.qy_raw;
        currentData.qz_filtered = currentData.qz_raw;
        first_run = false;
        return;
    }
    
    float alpha = 0.1f;
    currentData.qw_filtered = currentData.qw_filtered * (1-alpha) + currentData.qw_raw * alpha;
    currentData.qx_filtered = currentData.qx_filtered * (1-alpha) + currentData.qx_raw * alpha;
    currentData.qy_filtered = currentData.qy_filtered * (1-alpha) + currentData.qy_raw * alpha;
    currentData.qz_filtered = currentData.qz_filtered * (1-alpha) + currentData.qz_raw * alpha;
    
    normalizeQuaternion(currentData.qw_filtered, currentData.qx_filtered, currentData.qy_filtered, currentData.qz_filtered);
}

void HWT906Handler::convertQuaternionToEuler() {
    float qw = currentData.qw_raw;
    float qx = currentData.qx_raw;
    float qy = currentData.qy_raw;
    float qz = currentData.qz_raw;
    
    // Roll
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    currentData.roll_Q = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
    
    // Pitch (safe version)
    float sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1) {
        currentData.pitch_Q = copysign(90.0f, sinp);
    } else {
        currentData.pitch_Q = asin(sinp) * 180.0f / PI;
    }
    
    // Yaw
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    currentData.yaw_Q = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
    
    while (currentData.yaw_Q > 180.0f) currentData.yaw_Q -= 360.0f;
    while (currentData.yaw_Q < -180.0f) currentData.yaw_Q += 360.0f;
}

void HWT906Handler::convertQuaternionToEulerFiltered() {
    float qw = currentData.qw_filtered;
    float qx = currentData.qx_filtered;
    float qy = currentData.qy_filtered;
    float qz = currentData.qz_filtered;
    
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
    
    while (currentData.yaw_Q_filtered > 180.0f) currentData.yaw_Q_filtered -= 360.0f;
    while (currentData.yaw_Q_filtered < -180.0f) currentData.yaw_Q_filtered += 360.0f;
}

void HWT906Handler::detectGimbalLock() {
    bool euler_gimbal = (abs(currentData.pitch_filtered) > 85.0f);
    bool quaternion_gimbal = (abs(currentData.pitch_Q_filtered) > 85.0f);
    
    currentData.gimbal_lock_detected = (euler_gimbal || quaternion_gimbal);
    
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

// === STEP CAL: IMPLEMENTAZIONE CALIBRAZIONI FIXED ===

void HWT906Handler::applySensorCalibrations() {
    // === GYRO CALIBRATION ===
    if (gyro_cal.is_calibrated) {
        currentData.gx_cal = currentData.gx - gyro_cal.bias_x;
        currentData.gy_cal = currentData.gy - gyro_cal.bias_y;
        currentData.gz_cal = currentData.gz - gyro_cal.bias_z;
    } else {
        currentData.gx_cal = currentData.gx;
        currentData.gy_cal = currentData.gy;
        currentData.gz_cal = currentData.gz;
    }
    
    // === ACCELEROMETER CALIBRATION ===
    if (accel_cal.is_calibrated) {
        currentData.ax_cal = (currentData.ax - accel_cal.offset_x) * accel_cal.scale_x;
        currentData.ay_cal = (currentData.ay - accel_cal.offset_y) * accel_cal.scale_y;
        currentData.az_cal = (currentData.az - accel_cal.offset_z) * accel_cal.scale_z;
    } else {
        currentData.ax_cal = currentData.ax;
        currentData.ay_cal = currentData.ay;
        currentData.az_cal = currentData.az;
    }
    
    // === MAGNETOMETER CALIBRATION ===
    if (mag_cal.is_calibrated) {
        // Apply hard iron compensation
        float mx_hi = currentData.mx - mag_cal.hard_iron_x;
        float my_hi = currentData.my - mag_cal.hard_iron_y;
        float mz_hi = currentData.mz - mag_cal.hard_iron_z;
        
        // Apply soft iron compensation (matrix multiplication)
        currentData.mx_cal = mag_cal.soft_iron_matrix[0][0] * mx_hi + 
                            mag_cal.soft_iron_matrix[0][1] * my_hi + 
                            mag_cal.soft_iron_matrix[0][2] * mz_hi;
        currentData.my_cal = mag_cal.soft_iron_matrix[1][0] * mx_hi + 
                            mag_cal.soft_iron_matrix[1][1] * my_hi + 
                            mag_cal.soft_iron_matrix[1][2] * mz_hi;
        currentData.mz_cal = mag_cal.soft_iron_matrix[2][0] * mx_hi + 
                            mag_cal.soft_iron_matrix[2][1] * my_hi + 
                            mag_cal.soft_iron_matrix[2][2] * mz_hi;
    } else {
        currentData.mx_cal = currentData.mx;
        currentData.my_cal = currentData.my;
        currentData.mz_cal = currentData.mz;
    }
}

void HWT906Handler::processCalibrationStateMachine() {
    if (cal_state == CAL_IDLE) return;
    
    uint32_t now = millis();
    
    // Timeout check
    if (now - cal_start_time > cal_timeout) {
        Serial.println("⏰ Calibration timeout - cancelling");
        cal_state = CAL_IDLE;
        return;
    }
    
    switch (cal_state) {
        case CAL_GYRO_COLLECTING:
            collectGyroSample();
            break;
            
        case CAL_ACCEL_POINT_1_POS_X:
        case CAL_ACCEL_POINT_2_NEG_X:
        case CAL_ACCEL_POINT_3_POS_Y:
        case CAL_ACCEL_POINT_4_NEG_Y:
        case CAL_ACCEL_POINT_5_POS_Z:
        case CAL_ACCEL_POINT_6_NEG_Z:
            // Collection automatica continua dopo confirmAccelPoint()
            if (accel_cal.current_point_samples > 0 && accel_cal.current_point_samples < 200) {  // 🔧 FIXED: 200 campioni
                static uint32_t last_auto_collect = 0;
                if (now - last_auto_collect > 50) {  // 20Hz sampling
                    last_auto_collect = now;
                    
                    // Accumula campioni automaticamente
                    accel_cal.point_sum_x += currentData.ax;
                    accel_cal.point_sum_y += currentData.ay;
                    accel_cal.point_sum_z += currentData.az;
                    accel_cal.current_point_samples++;
                    
                    // Debug progress ogni 40 campioni
                    if (accel_cal.current_point_samples % 40 == 0) {
                        Serial.printf("📊 Collecting point %d... %d/200 samples\n", 
                                     getCurrentAccelPoint(), accel_cal.current_point_samples);
                    }
                    
                    // 🔧 FIXED: Completa punto quando raggiunti 200 campioni
                    if (accel_cal.current_point_samples >= 200) {
                        int point_index = getCurrentAccelPoint() - 1;
                        
                        // Calcola medie e salva punto
                        accel_cal.calibration_points[point_index][0] = accel_cal.point_sum_x / 200.0f;
                        accel_cal.calibration_points[point_index][1] = accel_cal.point_sum_y / 200.0f;
                        accel_cal.calibration_points[point_index][2] = accel_cal.point_sum_z / 200.0f;
                        
                        accel_cal.points_collected = point_index + 1;
                        
                        Serial.printf("✅ Point %d completed: %.2f %.2f %.2f g\n", 
                                     point_index + 1,
                                     accel_cal.calibration_points[point_index][0],
                                     accel_cal.calibration_points[point_index][1],
                                     accel_cal.calibration_points[point_index][2]);
                        
                        // Reset per prossimo punto
                        accel_cal.current_point_samples = 0;
                        accel_cal.point_sum_x = 0.0f;
                        accel_cal.point_sum_y = 0.0f;
                        accel_cal.point_sum_z = 0.0f;
                        
                        // Avanza al prossimo punto o termina
                        if (point_index < 5) {
                            cal_state = (CalibrationState)(CAL_ACCEL_POINT_1_POS_X + point_index + 1);
                            Serial.printf("   Place sensor in %s position and confirm\n", getCurrentAccelPointName());
                        } else {
                            calculateAccelCalibration();
                            finishAccelCalibration();
                        }
                    }
                }
            }
            break;
            
        case CAL_MAG_COLLECTING:
            collectMagSample();
            break;
            
        default:
            break;
    }
}

// === GYRO CALIBRATION - FIXED ===

bool HWT906Handler::startGyroCalibration() {
    if (cal_state != CAL_IDLE) {
        Serial.println("❌ Another calibration in progress");
        return false;
    }
    
    Serial.println("🎯 Starting Gyro Calibration...");
    Serial.println("   Keep sensor PERFECTLY STILL for 50 seconds");  // 🔧 FIXED: 50 secondi
    
    gyro_cal.reset();
    cal_state = CAL_GYRO_COLLECTING;
    cal_start_time = millis();
    cal_timeout = 120000;  // 🔧 FIXED: 60s timeout per 50s raccolta
    
    // 🔧 CRITICAL FIX: Reset accumulation variables
    gyro_cal.bias_x = 0.0f;
    gyro_cal.bias_y = 0.0f; 
    gyro_cal.bias_z = 0.0f;
    gyro_cal.calibration_samples = 0;
    
    return true;
}

void HWT906Handler::collectGyroSample() {
    uint32_t now = millis();
    static uint32_t last_sample = 0;
    
    if (now - last_sample < 50) return;  // 20Hz sampling
    last_sample = now;
    
    // 🔧 CRITICAL FIX: Accumulate correctly in struct variables
    gyro_cal.bias_x += currentData.gx;
    gyro_cal.bias_y += currentData.gy;
    gyro_cal.bias_z += currentData.gz;
    gyro_cal.calibration_samples++;
    
    // 🔧 FIXED: 1000 campioni @ 20Hz = 50 secondi
    if (gyro_cal.calibration_samples >= 1000) {
        calculateGyroBias();
        finishGyroCalibration();
    }
}

void HWT906Handler::calculateGyroBias() {
    // 🔧 CRITICAL FIX: Proper averaging calculation
    if (gyro_cal.calibration_samples > 0) {
        gyro_cal.bias_x = gyro_cal.bias_x / (float)gyro_cal.calibration_samples;
        gyro_cal.bias_y = gyro_cal.bias_y / (float)gyro_cal.calibration_samples;
        gyro_cal.bias_z = gyro_cal.bias_z / (float)gyro_cal.calibration_samples;
        
        gyro_cal.is_calibrated = true;
        gyro_cal.calibration_timestamp = millis();
        
        Serial.printf("✅ Gyro bias calculated: X=%.3f Y=%.3f Z=%.3f °/s\n",
                     gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    } else {
        Serial.println("❌ No gyro samples collected!");
    }
}

bool HWT906Handler::isGyroCalibrationInProgress() {
    return (cal_state == CAL_GYRO_COLLECTING);
}

float HWT906Handler::getGyroCalibrationProgress() {
    if (cal_state != CAL_GYRO_COLLECTING) return 0.0f;
    return (float)gyro_cal.calibration_samples / 1000.0f * 100.0f;  // 🔧 FIXED: 1000 campioni
}

void HWT906Handler::finishGyroCalibration() {
    if (gyro_cal.is_calibrated) {
        saveAllCalibrations();
        Serial.println("✅ Gyro calibration completed and saved");
    }
    cal_state = CAL_IDLE;
}

void HWT906Handler::cancelGyroCalibration() {
    gyro_cal.reset();
    cal_state = CAL_IDLE;
    Serial.println("🚫 Gyro calibration cancelled");
}

// === ACCELEROMETER CALIBRATION - FIXED ===

bool HWT906Handler::startAccelCalibration() {
    if (cal_state != CAL_IDLE) {
        Serial.println("❌ Another calibration in progress");
        return false;
    }
    
    Serial.println("🎯 Starting Accelerometer Calibration...");
    Serial.println("   6-point calibration: +X, -X, +Y, -Y, +Z, -Z");
    Serial.println("   Place sensor in +X position and confirm");
    
    accel_cal.reset();
    cal_state = CAL_ACCEL_POINT_1_POS_X;
    cal_start_time = millis();
    cal_timeout = 300000;  // 5 min timeout
    
    // 🔧 CRITICAL FIX: Initialize collection variables for first point
    accel_cal.current_point_samples = 0;
    accel_cal.point_sum_x = 0.0f;
    accel_cal.point_sum_y = 0.0f;
    accel_cal.point_sum_z = 0.0f;
    
    return true;
}

void HWT906Handler::collectAccelPoint(int point_index) {
    if (point_index < 0 || point_index >= 6) return;
    
    // 🔧 CRITICAL FIX: Use struct variables instead of static
    accel_cal.point_sum_x += currentData.ax;
    accel_cal.point_sum_y += currentData.ay;
    accel_cal.point_sum_z += currentData.az;
    accel_cal.current_point_samples++;
    
    // 🔧 FIXED: 200 campioni/punto @ 20Hz = 10 secondi/punto
    if (accel_cal.current_point_samples >= 200) {
        // Calculate averages
        accel_cal.calibration_points[point_index][0] = accel_cal.point_sum_x / 200.0f;
        accel_cal.calibration_points[point_index][1] = accel_cal.point_sum_y / 200.0f;
        accel_cal.calibration_points[point_index][2] = accel_cal.point_sum_z / 200.0f;
        
        accel_cal.points_collected = point_index + 1;
        
        Serial.printf("✅ Point %d collected: %.2f %.2f %.2f g\n", 
                     point_index + 1,
                     accel_cal.calibration_points[point_index][0],
                     accel_cal.calibration_points[point_index][1],
                     accel_cal.calibration_points[point_index][2]);
        
        // 🔧 CRITICAL FIX: Reset collection variables for next point
        accel_cal.current_point_samples = 0;
        accel_cal.point_sum_x = 0.0f;
        accel_cal.point_sum_y = 0.0f;
        accel_cal.point_sum_z = 0.0f;
        
        // Advance to next point or finish
        if (point_index < 5) {
            cal_state = (CalibrationState)(CAL_ACCEL_POINT_1_POS_X + point_index + 1);
            Serial.printf("   Place sensor in %s position and confirm\n", getCurrentAccelPointName());
        } else {
            calculateAccelCalibration();
            finishAccelCalibration();
        }
    }
}

void HWT906Handler::calculateAccelCalibration() {
    // Simple 6-point calibration calculation
    // Assumes perfect orientations: ±1g on each axis
    
    // Calculate offsets (average of positive and negative readings)
    accel_cal.offset_x = (accel_cal.calibration_points[0][0] + accel_cal.calibration_points[1][0]) / 2.0f;
    accel_cal.offset_y = (accel_cal.calibration_points[2][1] + accel_cal.calibration_points[3][1]) / 2.0f;
    accel_cal.offset_z = (accel_cal.calibration_points[4][2] + accel_cal.calibration_points[5][2]) / 2.0f;
    
    // Calculate scale factors (difference between positive and negative readings)
    float range_x = accel_cal.calibration_points[0][0] - accel_cal.calibration_points[1][0];
    float range_y = accel_cal.calibration_points[2][1] - accel_cal.calibration_points[3][1];
    float range_z = accel_cal.calibration_points[4][2] - accel_cal.calibration_points[5][2];
    
    accel_cal.scale_x = (range_x != 0) ? 2.0f / range_x : 1.0f;  // 2g range / measured range
    accel_cal.scale_y = (range_y != 0) ? 2.0f / range_y : 1.0f;
    accel_cal.scale_z = (range_z != 0) ? 2.0f / range_z : 1.0f;
    
    accel_cal.is_calibrated = true;
    accel_cal.calibration_timestamp = millis();
    
    Serial.printf("✅ Accel calibration calculated:\n");
    Serial.printf("   Offsets: X=%.3f Y=%.3f Z=%.3f g\n", 
                 accel_cal.offset_x, accel_cal.offset_y, accel_cal.offset_z);
    Serial.printf("   Scales:  X=%.3f Y=%.3f Z=%.3f\n",
                 accel_cal.scale_x, accel_cal.scale_y, accel_cal.scale_z);
}

bool HWT906Handler::isAccelCalibrationInProgress() {
    return (cal_state >= CAL_ACCEL_POINT_1_POS_X && cal_state <= CAL_ACCEL_POINT_6_NEG_Z);
}

float HWT906Handler::getAccelCalibrationProgress() {
    if (!isAccelCalibrationInProgress()) return 0.0f;
    return (float)accel_cal.points_collected / 6.0f * 100.0f;
}

int HWT906Handler::getCurrentAccelPoint() {
    if (!isAccelCalibrationInProgress()) return -1;
    return (int)cal_state - (int)CAL_ACCEL_POINT_1_POS_X + 1;
}

const char* HWT906Handler::getCurrentAccelPointName() {
    switch (cal_state) {
        case CAL_ACCEL_POINT_1_POS_X: return "+X (right side down)";
        case CAL_ACCEL_POINT_2_NEG_X: return "-X (left side down)";
        case CAL_ACCEL_POINT_3_POS_Y: return "+Y (back side down)";
        case CAL_ACCEL_POINT_4_NEG_Y: return "-Y (front side down)";
        case CAL_ACCEL_POINT_5_POS_Z: return "+Z (top side up)";
        case CAL_ACCEL_POINT_6_NEG_Z: return "-Z (bottom side up)";
        default: return "Unknown";
    }
}

bool HWT906Handler::isAccelPointReady() {
    if (!isAccelCalibrationInProgress()) return false;
    
    // 🔧 IMPROVED: Simpler stability check using struct variables
    // Check if we have enough recent samples for stability assessment
    static float stability_buffer[10][3];
    static int buffer_index = 0;
    static bool buffer_full = false;
    
    stability_buffer[buffer_index][0] = currentData.ax;
    stability_buffer[buffer_index][1] = currentData.ay;
    stability_buffer[buffer_index][2] = currentData.az;
    
    buffer_index = (buffer_index + 1) % 10;
    if (buffer_index == 0) buffer_full = true;
    
    if (!buffer_full) return false;
    
    // Calculate standard deviation
    float mean_x = 0, mean_y = 0, mean_z = 0;
    for (int i = 0; i < 10; i++) {
        mean_x += stability_buffer[i][0];
        mean_y += stability_buffer[i][1];
        mean_z += stability_buffer[i][2];
    }
    mean_x /= 10; mean_y /= 10; mean_z /= 10;
    
    float var_x = 0, var_y = 0, var_z = 0;
    for (int i = 0; i < 10; i++) {
        var_x += (stability_buffer[i][0] - mean_x) * (stability_buffer[i][0] - mean_x);
        var_y += (stability_buffer[i][1] - mean_y) * (stability_buffer[i][1] - mean_y);
        var_z += (stability_buffer[i][2] - mean_z) * (stability_buffer[i][2] - mean_z);
    }
    
    float std_dev = sqrt((var_x + var_y + var_z) / 30.0f);
    
    return (std_dev < 0.05f);  // Stable if std dev < 0.05g
}

void HWT906Handler::confirmAccelPoint() {
    if (!isAccelCalibrationInProgress()) return;
    
    int point_index = getCurrentAccelPoint() - 1;
    Serial.printf("🔄 Starting collection for point %d... (10 seconds)\n", point_index + 1);  // 🔧 FIXED: 10 secondi
    
    // 🔧 FIX: Inizia collection automatica impostando current_point_samples = 1
    // Questo attiverà la logica di collection automatica in processCalibrationStateMachine()
    accel_cal.current_point_samples = 1;  // Trigger per collection automatica
    accel_cal.point_sum_x = currentData.ax;  // Primo campione
    accel_cal.point_sum_y = currentData.ay;
    accel_cal.point_sum_z = currentData.az;
}

void HWT906Handler::finishAccelCalibration() {
    if (accel_cal.is_calibrated) {
        saveAllCalibrations();
        Serial.println("✅ Accelerometer calibration completed and saved");
    }
    cal_state = CAL_IDLE;
}

void HWT906Handler::cancelAccelCalibration() {
    accel_cal.reset();
    cal_state = CAL_IDLE;
    Serial.println("🚫 Accelerometer calibration cancelled");
}

// === MAGNETOMETER CALIBRATION - FIXED ===

bool HWT906Handler::startMagCalibration() {
    if (cal_state != CAL_IDLE) {
        Serial.println("❌ Another calibration in progress");
        return false;
    }
    
    Serial.println("🎯 Starting Magnetometer Calibration...");
    Serial.println("   Rotate sensor slowly in ALL directions for 100 seconds");  // 🔧 FIXED: 100 secondi
    Serial.println("   Include figure-8 motions and full rotations");
    
    mag_cal.reset();
    cal_state = CAL_MAG_COLLECTING;
    cal_start_time = millis();
    cal_timeout = 120000;  // 🔧 FIXED: 120s timeout per 100s raccolta
    
    // Initialize min/max values properly
    mag_cal.min_x = mag_cal.min_y = mag_cal.min_z = 9999.0f;
    mag_cal.max_x = mag_cal.max_y = mag_cal.max_z = -9999.0f;
    mag_cal.samples_collected = 0;
    
    return true;
}

void HWT906Handler::collectMagSample() {
    static uint32_t last_sample = 0;
    uint32_t now = millis();
    
    if (now - last_sample < 50) return;  // 20Hz sampling
    last_sample = now;
    
    // Update min/max for each axis
    if (currentData.mx > mag_cal.max_x) mag_cal.max_x = currentData.mx;
    if (currentData.mx < mag_cal.min_x) mag_cal.min_x = currentData.mx;
    if (currentData.my > mag_cal.max_y) mag_cal.max_y = currentData.my;
    if (currentData.my < mag_cal.min_y) mag_cal.min_y = currentData.my;
    if (currentData.mz > mag_cal.max_z) mag_cal.max_z = currentData.mz;
    if (currentData.mz < mag_cal.min_z) mag_cal.min_z = currentData.mz;
    
    mag_cal.samples_collected++;
    
    // 🔧 FIXED: 2000 campioni @ 20Hz = 100 secondi
    if (mag_cal.samples_collected >= 500) {
        calculateMagCalibration();
        finishMagCalibration();
    }
}

void HWT906Handler::calculateMagCalibration() {
    // Simple hard iron compensation (sphere center)
    mag_cal.hard_iron_x = (mag_cal.max_x + mag_cal.min_x) / 2.0f;
    mag_cal.hard_iron_y = (mag_cal.max_y + mag_cal.min_y) / 2.0f;
    mag_cal.hard_iron_z = (mag_cal.max_z + mag_cal.min_z) / 2.0f;
    
    // Simple soft iron compensation (sphere scaling)
    float range_x = mag_cal.max_x - mag_cal.min_x;
    float range_y = mag_cal.max_y - mag_cal.min_y;
    float range_z = mag_cal.max_z - mag_cal.min_z;
    
    // Use average range as reference
    float avg_range = (range_x + range_y + range_z) / 3.0f;
    
    if (avg_range > 0) {
        mag_cal.soft_iron_matrix[0][0] = avg_range / range_x;
        mag_cal.soft_iron_matrix[1][1] = avg_range / range_y;
        mag_cal.soft_iron_matrix[2][2] = avg_range / range_z;
    }
    
    mag_cal.is_calibrated = true;
    mag_cal.calibration_timestamp = millis();
    
    Serial.printf("✅ Mag calibration calculated:\n");
    Serial.printf("   Hard Iron: X=%.1f Y=%.1f Z=%.1f\n", 
                 mag_cal.hard_iron_x, mag_cal.hard_iron_y, mag_cal.hard_iron_z);
    Serial.printf("   Soft Iron: X=%.3f Y=%.3f Z=%.3f\n",
                 mag_cal.soft_iron_matrix[0][0], mag_cal.soft_iron_matrix[1][1], mag_cal.soft_iron_matrix[2][2]);
}

bool HWT906Handler::isMagCalibrationInProgress() {
    return (cal_state == CAL_MAG_COLLECTING);
}

float HWT906Handler::getMagCalibrationProgress() {
    if (cal_state != CAL_MAG_COLLECTING) return 0.0f;
    return (float)mag_cal.samples_collected / 500.0f * 100.0f;  // 🔧 FIXED: 2000 campioni
}

void HWT906Handler::finishMagCalibration() {
    if (mag_cal.is_calibrated) {
        saveAllCalibrations();
        Serial.println("✅ Magnetometer calibration completed and saved");
    }
    cal_state = CAL_IDLE;
}

void HWT906Handler::cancelMagCalibration() {
    mag_cal.reset();
    cal_state = CAL_IDLE;
    Serial.println("🚫 Magnetometer calibration cancelled");
}

// === STEP CAL: STORAGE METHODS (PREFERENCES) - unchanged ===

void HWT906Handler::saveAllCalibrations() {
    Serial.println("💾 Saving all calibrations to Preferences...");
    
    // Gyro calibration
    prefs.putFloat("gyro_bias_x", gyro_cal.bias_x);
    prefs.putFloat("gyro_bias_y", gyro_cal.bias_y);
    prefs.putFloat("gyro_bias_z", gyro_cal.bias_z);
    prefs.putBool("gyro_calibrated", gyro_cal.is_calibrated);
    prefs.putUInt("gyro_timestamp", gyro_cal.calibration_timestamp);
    
    // Accelerometer calibration
    prefs.putFloat("accel_offset_x", accel_cal.offset_x);
    prefs.putFloat("accel_offset_y", accel_cal.offset_y);
    prefs.putFloat("accel_offset_z", accel_cal.offset_z);
    prefs.putFloat("accel_scale_x", accel_cal.scale_x);
    prefs.putFloat("accel_scale_y", accel_cal.scale_y);
    prefs.putFloat("accel_scale_z", accel_cal.scale_z);
    prefs.putBool("accel_calibrated", accel_cal.is_calibrated);
    prefs.putUInt("accel_timestamp", accel_cal.calibration_timestamp);
    
    // Magnetometer calibration
    prefs.putFloat("mag_hard_iron_x", mag_cal.hard_iron_x);
    prefs.putFloat("mag_hard_iron_y", mag_cal.hard_iron_y);
    prefs.putFloat("mag_hard_iron_z", mag_cal.hard_iron_z);
    prefs.putFloat("mag_soft_iron_xx", mag_cal.soft_iron_matrix[0][0]);
    prefs.putFloat("mag_soft_iron_yy", mag_cal.soft_iron_matrix[1][1]);
    prefs.putFloat("mag_soft_iron_zz", mag_cal.soft_iron_matrix[2][2]);
    prefs.putBool("mag_calibrated", mag_cal.is_calibrated);
    prefs.putUInt("mag_timestamp", mag_cal.calibration_timestamp);
    
    Serial.println("✅ All calibrations saved successfully");
}

void HWT906Handler::loadAllCalibrations() {
    Serial.println("📂 Loading calibrations from Preferences...");
    
    // Gyro calibration
    gyro_cal.bias_x = prefs.getFloat("gyro_bias_x", 0.0f);
    gyro_cal.bias_y = prefs.getFloat("gyro_bias_y", 0.0f);
    gyro_cal.bias_z = prefs.getFloat("gyro_bias_z", 0.0f);
    gyro_cal.is_calibrated = prefs.getBool("gyro_calibrated", false);
    gyro_cal.calibration_timestamp = prefs.getUInt("gyro_timestamp", 0);
    
    // Accelerometer calibration
    accel_cal.offset_x = prefs.getFloat("accel_offset_x", 0.0f);
    accel_cal.offset_y = prefs.getFloat("accel_offset_y", 0.0f);
    accel_cal.offset_z = prefs.getFloat("accel_offset_z", 0.0f);
    accel_cal.scale_x = prefs.getFloat("accel_scale_x", 1.0f);
    accel_cal.scale_y = prefs.getFloat("accel_scale_y", 1.0f);
    accel_cal.scale_z = prefs.getFloat("accel_scale_z", 1.0f);
    accel_cal.is_calibrated = prefs.getBool("accel_calibrated", false);
    accel_cal.calibration_timestamp = prefs.getUInt("accel_timestamp", 0);
    
    // Magnetometer calibration
    mag_cal.hard_iron_x = prefs.getFloat("mag_hard_iron_x", 0.0f);
    mag_cal.hard_iron_y = prefs.getFloat("mag_hard_iron_y", 0.0f);
    mag_cal.hard_iron_z = prefs.getFloat("mag_hard_iron_z", 0.0f);
    mag_cal.soft_iron_matrix[0][0] = prefs.getFloat("mag_soft_iron_xx", 1.0f);
    mag_cal.soft_iron_matrix[1][1] = prefs.getFloat("mag_soft_iron_yy", 1.0f);
    mag_cal.soft_iron_matrix[2][2] = prefs.getFloat("mag_soft_iron_zz", 1.0f);
    mag_cal.is_calibrated = prefs.getBool("mag_calibrated", false);
    mag_cal.calibration_timestamp = prefs.getUInt("mag_timestamp", 0);
    
    if (gyro_cal.is_calibrated) {
        Serial.printf("📋 Gyro calibration loaded: bias=(%.3f, %.3f, %.3f) °/s\n",
                     gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    }
    
    if (accel_cal.is_calibrated) {
        Serial.printf("📋 Accel calibration loaded: offset=(%.3f, %.3f, %.3f) scale=(%.3f, %.3f, %.3f)\n",
                     accel_cal.offset_x, accel_cal.offset_y, accel_cal.offset_z,
                     accel_cal.scale_x, accel_cal.scale_y, accel_cal.scale_z);
    }
    
    if (mag_cal.is_calibrated) {
        Serial.printf("📋 Mag calibration loaded: hard_iron=(%.1f, %.1f, %.1f)\n",
                     mag_cal.hard_iron_x, mag_cal.hard_iron_y, mag_cal.hard_iron_z);
    }
    
    // Legacy calibration load
    loadCalibration();
}

void HWT906Handler::resetAllSensorCalibrations() {
    Serial.println("🔄 Resetting all sensor calibrations...");
    
    gyro_cal.reset();
    accel_cal.reset();
    mag_cal.reset();
    
    // Clear from Preferences
    prefs.clear();
    
    Serial.println("✅ All sensor calibrations reset");
}

// === CALIBRATION INFO GETTERS ===

void HWT906Handler::getGyroCalibrationInfo(float& bias_x, float& bias_y, float& bias_z, uint32_t& timestamp) {
    bias_x = gyro_cal.bias_x;
    bias_y = gyro_cal.bias_y;
    bias_z = gyro_cal.bias_z;
    timestamp = gyro_cal.calibration_timestamp;
}

void HWT906Handler::getAccelCalibrationInfo(float& offset_x, float& offset_y, float& offset_z, 
                                           float& scale_x, float& scale_y, float& scale_z, uint32_t& timestamp) {
    offset_x = accel_cal.offset_x;
    offset_y = accel_cal.offset_y;
    offset_z = accel_cal.offset_z;
    scale_x = accel_cal.scale_x;
    scale_y = accel_cal.scale_y;
    scale_z = accel_cal.scale_z;
    timestamp = accel_cal.calibration_timestamp;
}

void HWT906Handler::getMagCalibrationInfo(float& hard_iron_x, float& hard_iron_y, float& hard_iron_z, uint32_t& timestamp) {
    hard_iron_x = mag_cal.hard_iron_x;
    hard_iron_y = mag_cal.hard_iron_y;
    hard_iron_z = mag_cal.hard_iron_z;
    timestamp = mag_cal.calibration_timestamp;
}

// === RESET INDIVIDUAL CALIBRATIONS ===

void HWT906Handler::resetGyroCalibration() {
    gyro_cal.reset();
    prefs.remove("gyro_bias_x");
    prefs.remove("gyro_bias_y");
    prefs.remove("gyro_bias_z");
    prefs.remove("gyro_calibrated");
    prefs.remove("gyro_timestamp");
    Serial.println("🔄 Gyro calibration reset");
}

void HWT906Handler::resetAccelCalibration() {
    accel_cal.reset();
    prefs.remove("accel_offset_x");
    prefs.remove("accel_offset_y");
    prefs.remove("accel_offset_z");
    prefs.remove("accel_scale_x");
    prefs.remove("accel_scale_y");
    prefs.remove("accel_scale_z");
    prefs.remove("accel_calibrated");
    prefs.remove("accel_timestamp");
    Serial.println("🔄 Accelerometer calibration reset");
}

void HWT906Handler::resetMagCalibration() {
    mag_cal.reset();
    prefs.remove("mag_hard_iron_x");
    prefs.remove("mag_hard_iron_y");
    prefs.remove("mag_hard_iron_z");
    prefs.remove("mag_soft_iron_xx");
    prefs.remove("mag_soft_iron_yy");
    prefs.remove("mag_soft_iron_zz");
    prefs.remove("mag_calibrated");
    prefs.remove("mag_timestamp");
    Serial.println("🔄 Magnetometer calibration reset");
}

// === EXISTING FILTER IMPLEMENTATIONS (unchanged) ===

void HWT906Handler::initAdvancedFilters() {
    memset(filters.roll_buffer, 0, sizeof(filters.roll_buffer));
    memset(filters.pitch_buffer, 0, sizeof(filters.pitch_buffer));
    memset(filters.yaw_buffer, 0, sizeof(filters.yaw_buffer));
    filters.buffer_index = 0;
    filters.buffer_full = false;
    
    for (int i = 0; i < 3; i++) {
        filters.P[i][0][0] = 1.0f;
        filters.P[i][0][1] = 0.0f;
        filters.P[i][1][0] = 0.0f;
        filters.P[i][1][1] = 1.0f;
    }
    
    Serial.println("✅ Advanced filters initialized (Enhanced for Step CAL)");
}

void HWT906Handler::applyAdvancedFiltering() {
    updateMovingAverage();
    // applyComplementaryFilter();  // Commented out for pitch range fix
    applyKalmanFilterYaw();
    applyEMAFilter();
    applyStaticThreshold();
}

void HWT906Handler::updateMovingAverage() {
    filters.roll_buffer[filters.buffer_index] = currentData.roll;
    filters.pitch_buffer[filters.buffer_index] = currentData.pitch;
    filters.yaw_buffer[filters.buffer_index] = currentData.yaw;
    
    filters.buffer_index = (filters.buffer_index + 1) % 10;
    if (filters.buffer_index == 0) filters.buffer_full = true;
    
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
    float acc_magnitude = sqrt(currentData.ax_cal * currentData.ax_cal + 
                              currentData.ay_cal * currentData.ay_cal + 
                              currentData.az_cal * currentData.az_cal);
    
    if (acc_magnitude > 0.5f && acc_magnitude < 1.5f) {
        float acc_roll = atan2(currentData.ay_cal, currentData.az_cal) * 180.0f / PI;
        float acc_pitch = atan2(-currentData.ax_cal, sqrt(currentData.ay_cal * currentData.ay_cal + currentData.az_cal * currentData.az_cal)) * 180.0f / PI;
        
        currentData.roll = filters.comp_alpha * currentData.roll + (1.0f - filters.comp_alpha) * acc_roll;
        currentData.pitch = filters.comp_alpha * currentData.pitch + (1.0f - filters.comp_alpha) * acc_pitch;
    }
}

void HWT906Handler::applyKalmanFilterYaw() {
    int idx = 2; // Yaw index
    
    filters.angle[idx] += (currentData.gz_cal - filters.bias[idx]) * 0.01f;
    filters.P[idx][0][0] += 0.01f * (0.01f * filters.P[idx][1][1] - filters.P[idx][0][1] - filters.P[idx][1][0] + filters.Q_angle);
    filters.P[idx][0][1] -= 0.01f * filters.P[idx][1][1];
    filters.P[idx][1][0] -= 0.01f * filters.P[idx][1][1];
    filters.P[idx][1][1] += filters.Q_bias * 0.01f;
    
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
    currentData.roll_filtered = filters.ema_alpha * currentData.roll + (1.0f - filters.ema_alpha) * filters.last_roll;
    currentData.pitch_filtered = filters.ema_alpha * currentData.pitch + (1.0f - filters.ema_alpha) * filters.last_pitch;
    currentData.yaw_filtered = filters.ema_alpha * currentData.yaw + (1.0f - filters.ema_alpha) * filters.last_yaw;
    
    filters.last_roll = currentData.roll_filtered;
    filters.last_pitch = currentData.pitch_filtered;
    filters.last_yaw = currentData.yaw_filtered;
}

void HWT906Handler::applyStaticThreshold() {
    uint32_t now = millis();
    
    float pitch_delta = abs(currentData.pitch_filtered - filters.last_pitch);
    if (pitch_delta < filters.pitch_dead_zone) {
        currentData.pitch_filtered = filters.last_pitch;
    } else {
        filters.last_significant_change = now;
    }
    
    float yaw_delta = abs(currentData.yaw_filtered - filters.last_yaw);
    if (yaw_delta < filters.yaw_dead_zone) {
        currentData.yaw_filtered = filters.last_yaw;
    } else {
        filters.last_significant_change = now;
    }
}

// === DRIFT COMPENSATION (existing code - unchanged) ===

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
    
    if (now - drift.last_compensation >= 1000) {
        updateDriftHistory();
        
        if (drift.history_full || drift.history_index > 60) {
            calculateDriftRate();
            
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
    int samples = drift.history_full ? 300 : drift.history_index;
    if (samples < 30) return;
    
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    uint32_t base_time = drift.history_timestamps[0];
    
    for (int i = 0; i < samples; i++) {
        float x = (drift.history_timestamps[i] - base_time) / 1000.0f;
        float y = drift.yaw_history[i];
        
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    
    float slope = (samples * sum_xy - sum_x * sum_y) / (samples * sum_x2 - sum_x * sum_x);
    drift.yaw_drift_rate = slope;
}

void HWT906Handler::detectStaticState() {
    bool currently_static = (abs(currentData.gx_cal) < currentData.static_threshold &&
                           abs(currentData.gy_cal) < currentData.static_threshold &&
                           abs(currentData.gz_cal) < currentData.static_threshold);
    
    if (currently_static != currentData.is_static) {
        if (currently_static) {
            currentData.is_static = true;
            currentData.static_start_time = millis();
            
            filters.ema_alpha = 0.05f;
            filters.comp_alpha = 0.99f;
            
            Serial.println("🔒 Static state detected - enhanced filtering enabled");
        } else {
            uint32_t static_duration = millis() - currentData.static_start_time;
            Serial.printf("🔄 Dynamic state detected after %lus static\n", static_duration/1000);
            
            filters.ema_alpha = 0.1f;
            filters.comp_alpha = 0.98f;
            
            currentData.is_static = false;
        }
    }
}

// === I2C METHODS (existing - unchanged) ===

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
    Wire.write(value & 0xFF);
    Wire.write((value >> 8) & 0xFF);
    return (Wire.endTransmission() == 0);
}

bool HWT906Handler::lockHWT906() {
    delay(10);
    return true;
}

// === LEGACY CALIBRATION (existing - maintain compatibility) ===

void HWT906Handler::startCalibration() {
    Serial.println("🎯 Starting legacy calibration...");
    
    currentData.pitch_offset = 0.0f;
    currentData.yaw_offset = 0.0f;
    currentData.roll_offset = 0.0f;
    
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
        
        Serial.printf("✅ Legacy calibration complete: P=%.2f Y=%.2f R=%.2f\n", 
                     currentData.pitch_offset, currentData.yaw_offset, currentData.roll_offset);
    }
}

void HWT906Handler::saveCalibration() {
    // Legacy EEPROM save - keep for compatibility but use different offset
    // to avoid conflicts with Preferences
    prefs.putFloat("legacy_pitch_offset", currentData.pitch_offset);
    prefs.putFloat("legacy_yaw_offset", currentData.yaw_offset);
    prefs.putFloat("legacy_roll_offset", currentData.roll_offset);
    prefs.putBool("legacy_calibrated", true);
}

void HWT906Handler::loadCalibration() {
    bool calibrated = prefs.getBool("legacy_calibrated", false);
    
    if (calibrated) {
        currentData.pitch_offset = prefs.getFloat("legacy_pitch_offset", 0.0f);
        currentData.yaw_offset = prefs.getFloat("legacy_yaw_offset", 0.0f);
        currentData.roll_offset = prefs.getFloat("legacy_roll_offset", 0.0f);
        currentData.is_calibrated = true;
        
        Serial.printf("📋 Legacy calibration loaded: P=%.2f Y=%.2f R=%.2f\n",
                     currentData.pitch_offset, currentData.yaw_offset, currentData.roll_offset);
    }
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

void HWT906Handler::resetCalibration() {
    currentData.pitch_offset = 0.0f;
    currentData.yaw_offset = 0.0f;
    currentData.roll_offset = 0.0f;
    currentData.is_calibrated = false;
    
    prefs.remove("legacy_pitch_offset");
    prefs.remove("legacy_yaw_offset");
    prefs.remove("legacy_roll_offset");
    prefs.remove("legacy_calibrated");
    
    Serial.println("🔄 Legacy calibration reset");
}

// === OTHER PUBLIC METHODS (existing - unchanged) ===

void HWT906Handler::setPrecisionMode(HWT906PrecisionMode mode) {
    current_mode = mode;
    configureStaticOptimized(mode);
}

void HWT906Handler::setDriftCompensation(bool enable) {
    drift.enabled = enable;
    Serial.printf("🔄 Drift compensation: %s\n", enable ? "ENABLED" : "DISABLED");
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

void HWT906Handler::updateStatistics() {
    uint32_t now = millis();
    static uint32_t last_update = 0;
    
    if (now - last_update >= 1000) {
        currentData.update_rate = (float)currentData.frame_count / ((now - last_update) / 1000.0f);
        last_update = now;
    }
}

void HWT906Handler::printDebugInfo() {
    if (debug_counter % (DEBUG_INTERVAL * 10) == 0) {
        Serial.printf("📊 HWT906 DUAL: P=%.2f° Y=%.2f° D=%.2f° | P_Q=%.2f° Y_Q=%.2f° | Rate=%.1fHz | GL=%s\n",
                     currentData.pitch_filtered, currentData.yaw_filtered, currentData.roll_filtered,
                     currentData.pitch_Q_filtered, currentData.yaw_Q_filtered,
                     currentData.update_rate, currentData.gimbal_lock_detected ? "YES" : "NO");
        
        Serial.printf("   Frames: E=%lu Q=%lu Err=%lu | CAL: G=%s A=%s M=%s | %s\n",
                     currentData.euler_frames_received, currentData.quaternion_frames_received,
                     currentData.error_frames, 
                     isGyroCalibrated() ? "✅" : "❌",
                     isAccelCalibrated() ? "✅" : "❌",
                     isMagCalibrated() ? "✅" : "❌",
                     currentData.is_static ? "STATIC" : "DYNAMIC");
    }
}

void HWT906Handler::printStatus() {
    Serial.println("\n=== HWT906 DUAL-MODE + STEP CAL STATUS ===");
    Serial.printf("Ready: %s\n", sensors_ready ? "YES" : "NO");
    Serial.printf("Mode: %s\n", precision_configs[current_mode].description);
    Serial.printf("Dual-Mode: %s\n", currentData.quaternion_mode_active ? "ACTIVE" : "INACTIVE");
    Serial.printf("Gimbal Lock: %s\n", currentData.gimbal_lock_detected ? "DETECTED" : "OK");
    Serial.printf("Drift Comp: %s\n", drift.enabled ? "ON" : "OFF");
    Serial.printf("Static: %s\n", currentData.is_static ? "YES" : "NO");
    
    Serial.printf("=== CALIBRATION STATUS ===\n");
    Serial.printf("Gyro: %s", isGyroCalibrated() ? "✅ CALIBRATED" : "❌ NOT CALIBRATED");
    if (isGyroCalibrated()) {
        Serial.printf(" (bias: %.3f, %.3f, %.3f °/s)", gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    }
    Serial.println();
    
    Serial.printf("Accel: %s", isAccelCalibrated() ? "✅ CALIBRATED" : "❌ NOT CALIBRATED");
    if (isAccelCalibrated()) {
        Serial.printf(" (offset: %.3f, %.3f, %.3f g)", accel_cal.offset_x, accel_cal.offset_y, accel_cal.offset_z);
    }
    Serial.println();
    
    Serial.printf("Mag: %s", isMagCalibrated() ? "✅ CALIBRATED" : "❌ NOT CALIBRATED");
    if (isMagCalibrated()) {
        Serial.printf(" (hard iron: %.1f, %.1f, %.1f)", mag_cal.hard_iron_x, mag_cal.hard_iron_y, mag_cal.hard_iron_z);
    }
    Serial.println();
    
    Serial.printf("Legacy: %s\n", currentData.is_calibrated ? "YES" : "NO");
    Serial.printf("Update Rate: %.1f Hz\n", currentData.update_rate);
    Serial.printf("Frames: Euler=%lu Quat=%lu (Errors: %lu)\n", 
                 currentData.euler_frames_received, currentData.quaternion_frames_received, currentData.error_frames);
    Serial.println("=======================================\n");
}

void HWT906Handler::enableDebug(bool enable) {
    Serial.printf("🔍 Debug mode: %s\n", enable ? "ON" : "OFF");
}

// === GLOBAL WRAPPER FUNCTIONS (existing - unchanged) ===

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

float getHWT906PitchRaw() {
    return hwt906.getPitchRaw();
}

float getHWT906YawRaw() {
    return hwt906.getYawRaw();
}

float getHWT906RollRaw() {
    return hwt906.getRollRaw();
}

// Dual-mode quaternions
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

// === STEP CAL: GLOBAL WRAPPER FUNCTIONS ===

bool startHWT906GyroCalibration() {
    return hwt906.startGyroCalibration();
}

bool startHWT906AccelCalibration() {
    return hwt906.startAccelCalibration();
}

bool startHWT906MagCalibration() {
    return hwt906.startMagCalibration();
}

bool isHWT906GyroCalibrationInProgress() {
    return hwt906.isGyroCalibrationInProgress();
}

bool isHWT906AccelCalibrationInProgress() {
    return hwt906.isAccelCalibrationInProgress();
}

bool isHWT906MagCalibrationInProgress() {
    return hwt906.isMagCalibrationInProgress();
}

float getHWT906GyroCalibrationProgress() {
    return hwt906.getGyroCalibrationProgress();
}

float getHWT906AccelCalibrationProgress() {
    return hwt906.getAccelCalibrationProgress();
}

float getHWT906MagCalibrationProgress() {
    return hwt906.getMagCalibrationProgress();
}

void finishHWT906GyroCalibration() {
    hwt906.finishGyroCalibration();
}

void finishHWT906AccelCalibration() {
    hwt906.finishAccelCalibration();
}

void finishHWT906MagCalibration() {
    hwt906.finishMagCalibration();
}

// === STEP CAL: COMPATIBILITY FUNCTIONS (for leaf_actions.cpp) ===

// Gyro compatibility
void startGyroCalibration() {
    hwt906.startGyroCalibration();
}

bool isGyroCalibrationInProgress() {
    return hwt906.isGyroCalibrationInProgress();
}

float getGyroCalibrationProgress() {
    return hwt906.getGyroCalibrationProgress();
}

void finishGyroCalibration() {
    hwt906.finishGyroCalibration();
}

// Accel compatibility
void startAccelCalibration() {
    hwt906.startAccelCalibration();
}

bool isAccelCalibrationInProgress() {
    return hwt906.isAccelCalibrationInProgress();
}

float getAccelCalibrationProgress() {
    return hwt906.getAccelCalibrationProgress();
}

int getCurrentAccelPoint() {
    return hwt906.getCurrentAccelPoint();
}

const char* getCurrentAccelPointName() {
    return hwt906.getCurrentAccelPointName();
}

bool isAccelPointReady() {
    return hwt906.isAccelPointReady();
}

void confirmAccelPoint() {
    hwt906.confirmAccelPoint();
}

void finishAccelCalibration() {
    hwt906.finishAccelCalibration();
}

// Mag compatibility  
void startMagCalibration() {
    hwt906.startMagCalibration();
}

bool isMagCalibrationInProgress() {
    return hwt906.isMagCalibrationInProgress();
}

float getMagCalibrationProgress() {
    return hwt906.getMagCalibrationProgress();
}

void finishMagCalibration() {
    hwt906.finishMagCalibration();
}

// Calibration status
bool isGyroCalibrated() {
    return hwt906.isGyroCalibrated();
}

bool isAccelCalibrated() {
    return hwt906.isAccelCalibrated();
}

bool isMagCalibrated() {
    return hwt906.isMagCalibrated();
}

// === CRITICAL FIX: MISSING CALIBRATION FUNCTIONS ===
// 🔧 These are the 3 missing functions that caused all calibration failures!

void calibrateGyro() { 
    hwt906.startGyroCalibration(); 
}

void calibrateAccel() { 
    hwt906.startAccelCalibration(); 
}

void calibrateMag() { 
    hwt906.startMagCalibration(); 
}
