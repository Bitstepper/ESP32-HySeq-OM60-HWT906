// hwt906_handler.h - Enhanced with Step CAL (Gyro + Accel + Mag Calibration)
// STEP CAL: Complete calibration system with Preferences.h storage
// Maintains all existing dual-mode quaternion functionality

#ifndef HWT906_HANDLER_H
#define HWT906_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>  // MIGRATED: EEPROM → Preferences for better storage

// === CONFIGURAZIONI OTTIMIZZATE ===
#define HWT906_I2C_ADDR 0x50
#define HWT906_UART_SPEED 9600
#define HWT906_PREFS_NAMESPACE "hwt906"  // Preferences namespace

// UART Frame Protocol
#define FRAME_HEADER 0x55
#define FRAME_ACCEL 0x51
#define FRAME_GYRO 0x52  
#define FRAME_ANGLE 0x53
#define FRAME_MAG 0x54
#define FRAME_QUATERNION 0x59
#define FRAME_SIZE 11

// === STEP CAL: CALIBRATION DATA STRUCTURES ===

struct GyroCalibrationData {
    float bias_x = 0.0f;
    float bias_y = 0.0f;
    float bias_z = 0.0f;
    bool is_calibrated = false;
    uint32_t calibration_timestamp = 0;
    int calibration_samples = 0;
    float temperature_at_calibration = 25.0f;
    
    void reset() {
        bias_x = bias_y = bias_z = 0.0f;
        is_calibrated = false;
        calibration_samples = 0;
    }
};

struct AccelCalibrationData {
    // 6-point calibration: +X, -X, +Y, -Y, +Z, -Z
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    float scale_x = 1.0f;
    float scale_y = 1.0f;
    float scale_z = 1.0f;
    bool is_calibrated = false;
    uint32_t calibration_timestamp = 0;
    float calibration_points[6][3];  // 6 orientations x 3 axes
    int points_collected = 0;
 
    // 🔧 AGGIUNGI QUESTE NUOVE VARIABILI:
    int current_point_samples;     // Campioni punto corrente
    float point_sum_x;            // Somma X punto corrente  
    float point_sum_y;            // Somma Y punto corrente
    float point_sum_z;            // Somma Z punto corrente 
    
    void reset() {
        offset_x = offset_y = offset_z = 0.0f;
        scale_x = scale_y = scale_z = 1.0f;
        is_calibrated = false;
        points_collected = 0;
        memset(calibration_points, 0, sizeof(calibration_points));
        
        // Reset nuove variabili
        current_point_samples = 0;
        point_sum_x = point_sum_y = point_sum_z = 0.0f;
        
    }
};

struct MagCalibrationData {
    // Hard iron and soft iron compensation
    float hard_iron_x = 0.0f;
    float hard_iron_y = 0.0f;
    float hard_iron_z = 0.0f;
    float soft_iron_matrix[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    bool is_calibrated = false;
    uint32_t calibration_timestamp = 0;
    float max_x = -32768, min_x = 32767;
    float max_y = -32768, min_y = 32767;
    float max_z = -32768, min_z = 32767;
    int samples_collected = 0;
    
    void reset() {
        hard_iron_x = hard_iron_y = hard_iron_z = 0.0f;
        // Reset to identity matrix
        memset(soft_iron_matrix, 0, sizeof(soft_iron_matrix));
        soft_iron_matrix[0][0] = soft_iron_matrix[1][1] = soft_iron_matrix[2][2] = 1.0f;
        is_calibrated = false;
        max_x = max_y = max_z = -32768;
        min_x = min_y = min_z = 32767;
        samples_collected = 0;
    }
};

// === CALIBRATION STATE MACHINE ===
enum CalibrationState {
    CAL_IDLE,
    CAL_GYRO_COLLECTING,
    CAL_ACCEL_POINT_1_POS_X,
    CAL_ACCEL_POINT_2_NEG_X,
    CAL_ACCEL_POINT_3_POS_Y,
    CAL_ACCEL_POINT_4_NEG_Y,
    CAL_ACCEL_POINT_5_POS_Z,
    CAL_ACCEL_POINT_6_NEG_Z,
    CAL_MAG_COLLECTING
};

// === PRECISIONE MODES ===
enum HWT906PrecisionMode {
    HWT906_MODE_STATIC_LOW = 0,
    HWT906_MODE_STATIC_MED = 1,
    HWT906_MODE_STATIC_HIGH = 2,
    HWT906_MODE_DYNAMIC = 3
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

// === FILTRI AVANZATI MULTI-STADIO ===
struct AdvancedFilters {
    float comp_alpha = 0.98f;
    
    float roll_buffer[10];
    float pitch_buffer[10];
    float yaw_buffer[10];
    int buffer_index = 0;
    bool buffer_full = false;
    
    float Q_angle = 0.001f;
    float Q_bias = 0.003f;
    float R_measure = 0.03f;
    
    float angle[3] = {0};
    float bias[3] = {0};
    float P[3][2][2];
    
    float ema_alpha = 0.1f;
    float last_roll = 0.0f;
    float last_pitch = 0.0f;
    float last_yaw = 0.0f;
    
    float yaw_dead_zone = 0.15f;
    float pitch_dead_zone = 0.1f;
    uint32_t last_significant_change = 0;
};

// === DRIFT COMPENSATION ===
struct DriftCompensation {
    bool enabled = true;
    float yaw_drift_rate = 0.0f;
    float yaw_reference = 0.0f;
    uint32_t last_compensation = 0;
    
    float yaw_history[300];    
    uint32_t history_timestamps[300];
    int history_index = 0;
    bool history_full = false;
    
    uint32_t reference_time = 0;
    const float MAX_COMP_RATE = 0.00033f;
};

// === STRUTTURA DATI PRINCIPALE (includes dual-mode) ===
struct HWT906Data {
    // Dati base 
    float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
    float pitch_raw = 0.0f, yaw_raw = 0.0f, roll_raw = 0.0f;
    float pitch_filtered = 0.0f, yaw_filtered = 0.0f, roll_filtered = 0.0f;
    
    // === DUAL-MODE QUATERNIONS ===
    float qw_raw = 1.0f, qx_raw = 0.0f, qy_raw = 0.0f, qz_raw = 0.0f;
    float qw_filtered = 1.0f, qx_filtered = 0.0f, qy_filtered = 0.0f, qz_filtered = 0.0f;
    float pitch_Q = 0.0f, yaw_Q = 0.0f, roll_Q = 0.0f;
    float pitch_Q_filtered = 0.0f, yaw_Q_filtered = 0.0f, roll_Q_filtered = 0.0f;
    
    bool quaternion_available = false;
    bool gimbal_lock_detected = false;
    bool quaternion_mode_active = false;
    uint32_t last_quaternion_time = 0;
    uint32_t quaternion_frames_received = 0;
    uint32_t euler_frames_received = 0;
    
    // Raw sensor data
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    
    // Calibrated sensor data
    float ax_cal = 0.0f, ay_cal = 0.0f, az_cal = 0.0f;
    float gx_cal = 0.0f, gy_cal = 0.0f, gz_cal = 0.0f;
    float mx_cal = 0.0f, my_cal = 0.0f, mz_cal = 0.0f;
    
    // Sistema
    bool is_static = false;
    uint32_t static_start_time = 0;
    float static_threshold = 0.5f;
    
    // Performance
    float update_rate = 0.0f;
    uint32_t frame_count = 0;
    uint32_t error_frames = 0;
    uint32_t last_frame_time = 0;
    
    // Range monitoring
    float pitch_min = 999.0f, pitch_max = -999.0f;
    float yaw_min = 999.0f, yaw_max = -999.0f;
    float roll_min = 999.0f, roll_max = -999.0f;
    
    // Legacy calibrazione (mantieni per compatibilità)
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
    
    // === STEP CAL: CALIBRATION DATA ===
    GyroCalibrationData gyro_cal;
    AccelCalibrationData accel_cal;
    MagCalibrationData mag_cal;
    CalibrationState cal_state = CAL_IDLE;
    uint32_t cal_start_time = 0;
    uint32_t cal_timeout = 30000;  // 30s timeout
    Preferences prefs;
    
    uint8_t frame_buffer[FRAME_SIZE];
    int frame_index = 0;
    bool frame_started = false;
    
    uint32_t last_valid_frame = 0;
    uint32_t debug_counter = 0;
    static const uint32_t DEBUG_INTERVAL = 50;
    
    HWT906PrecisionMode current_mode = HWT906_MODE_STATIC_MED;
    bool sensors_ready = false;
    
    HWT906PrecisionConfig precision_configs[4] = {
        {0x0005, 0x0006, 0x0064, 0x07D0, 0x0005, 0x1388, "Static Low - Max Filter"},
        {0x0006, 0x0005, 0x0050, 0x05DC, 0x0008, 0x0FA0, "Static Med - Balanced"},
        {0x0007, 0x0004, 0x003C, 0x03E8, 0x000A, 0x0BB8, "Static High - Responsive"},
        {0x0009, 0x0003, 0x0028, 0x01F4, 0x0010, 0x07D0, "Dynamic - Test Only"}
    };

    // === METODI PRIVATI ESISTENTI ===
    bool unlockHWT906();
    bool writeRegister(uint8_t reg, uint16_t value);
    bool lockHWT906();
    
    bool parseFrame(uint8_t* frame);
    bool validateFrame(uint8_t* frame);
    void processAngleFrame(uint8_t* frame);
    void processAccelFrame(uint8_t* frame);
    void processGyroFrame(uint8_t* frame);
    void processMagFrame(uint8_t* frame);
    
    // Dual-mode quaternions
    void processQuaternionFrame(uint8_t* frame);
    void convertQuaternionToEuler();
    void convertQuaternionToEulerFiltered();
    void applyQuaternionFilters();
    void detectGimbalLock();
    void normalizeQuaternion(float& qw, float& qx, float& qy, float& qz);
    
    // Filtri
    void initAdvancedFilters();
    void applyAdvancedFiltering();
    void updateMovingAverage();
    void applyComplementaryFilter();
    void applyKalmanFilterYaw();
    void applyEMAFilter();
    void applyStaticThreshold();
    void resetAllFilters();
    
    // Drift compensation
    void initDriftCompensation();
    void compensateDrift();
    void calculateDriftRate();
    void updateDriftHistory();
    void enableDriftCompensation(bool enable);
    
    void detectStaticState();
    void loadCalibration();  // Legacy
    void saveCalibration();  // Legacy
    void printDebugInfo();
    void updateStatistics();

    // === STEP CAL: METODI PRIVATI CALIBRAZIONE ===
    void applySensorCalibrations();
    void processCalibrationStateMachine();
    
    // Gyro calibration
    void collectGyroSample();
    void calculateGyroBias();
    
    // Accel calibration  
    void collectAccelPoint(int point_index);
    void calculateAccelCalibration();
    float calculateAccelMagnitude(float ax, float ay, float az);
    
    // Mag calibration
    void collectMagSample();
    void calculateMagCalibration();
    void calculateEllipsoidFit();
    
    // Storage methods
    void saveAllCalibrations();
    void loadAllCalibrations();
    void resetAllCalibrations();
    
    // 🔧 AGGIUNGI QUESTI DUE NUOVI METODI:
    void continuousAccelCollection();
    bool checkMagneticCoverage();

public:
    // === API PUBBLICA ESISTENTE ===
    bool init(HardwareSerial* serial_port = &Serial2);
    bool configureStaticOptimized(HWT906PrecisionMode mode = HWT906_MODE_STATIC_MED);
    
    void update();
    bool parseUARTData();
    
    // Getters principali
    float getPitch() { return currentData.pitch_filtered; }
    float getYaw() { return currentData.yaw_filtered; }
    float getRoll() { return currentData.roll_filtered; }
    
    float getPitchRaw() { return currentData.pitch_raw; }
    float getYawRaw() { return currentData.yaw_raw; }
    float getRollRaw() { return currentData.roll_raw; }
    
    // Dual-mode quaternions API
    float getQuaternionW() { return currentData.qw_raw; }
    float getQuaternionX() { return currentData.qx_raw; }
    float getQuaternionY() { return currentData.qy_raw; }
    float getQuaternionZ() { return currentData.qz_raw; }
    
    float getPitchQ() { return currentData.pitch_Q; }
    float getYawQ() { return currentData.yaw_Q; }
    float getRollQ() { return currentData.roll_Q; }
    
    float getPitchQFiltered() { return currentData.pitch_Q_filtered; }
    float getYawQFiltered() { return currentData.yaw_Q_filtered; }
    float getRollQFiltered() { return currentData.roll_Q_filtered; }
    
    bool isGimbalLockDetected() { return currentData.gimbal_lock_detected; }
    bool isQuaternionModeActive() { return currentData.quaternion_mode_active; }
    
    // === STEP CAL: API PUBBLICA CALIBRAZIONE ===
    
    // Gyro Calibration
    bool startGyroCalibration();
    bool isGyroCalibrationInProgress();
    float getGyroCalibrationProgress();
    void finishGyroCalibration();
    void cancelGyroCalibration();
    
    // Accelerometer Calibration  
    bool startAccelCalibration();
    bool isAccelCalibrationInProgress();
    float getAccelCalibrationProgress();
    int getCurrentAccelPoint();
    const char* getCurrentAccelPointName();
    bool isAccelPointReady();
    void confirmAccelPoint();
    void finishAccelCalibration();
    void cancelAccelCalibration();
    
    // Magnetometer Calibration
    bool startMagCalibration();
    bool isMagCalibrationInProgress();
    float getMagCalibrationProgress();
    void finishMagCalibration();
    void cancelMagCalibration();
    
    // Calibration Status
    bool isGyroCalibrated() { return gyro_cal.is_calibrated; }
    bool isAccelCalibrated() { return accel_cal.is_calibrated; }
    bool isMagCalibrated() { return mag_cal.is_calibrated; }
    bool isFullyCalibrated() { return isGyroCalibrated() && isAccelCalibrated() && isMagCalibrated(); }
    
    // Calibration Info
    void getGyroCalibrationInfo(float& bias_x, float& bias_y, float& bias_z, uint32_t& timestamp);
    void getAccelCalibrationInfo(float& offset_x, float& offset_y, float& offset_z, 
                                float& scale_x, float& scale_y, float& scale_z, uint32_t& timestamp);
    void getMagCalibrationInfo(float& hard_iron_x, float& hard_iron_y, float& hard_iron_z, uint32_t& timestamp);
    
    // Reset calibrations
    void resetGyroCalibration();
    void resetAccelCalibration(); 
    void resetMagCalibration();
    void resetAllSensorCalibrations();
    
    // Sistema
    bool isReady() { return sensors_ready; }
    bool isStatic() { return currentData.is_static; }
    float getUpdateRate() { return currentData.update_rate; }
    uint32_t getFrameCount() { return currentData.frame_count; }
    uint32_t getErrorFrames() { return currentData.error_frames; }
    
    // Legacy calibrazione (mantieni per compatibilità)
    void startCalibration();
    void saveCurrentAsZero();
    void resetCalibration();
    bool isCalibrated() { return currentData.is_calibrated; }
    
    // Configurazione
    void setPrecisionMode(HWT906PrecisionMode mode);
    void setDriftCompensation(bool enable);
    void setDeadZones(float pitch_zone, float yaw_zone);
    
    // Range e debug
    void getRange(float& pitch_min, float& pitch_max, float& yaw_min, float& yaw_max);
    void resetRange();
    void printStatus();
    void enableDebug(bool enable);
    
    const HWT906Data& getData() const { return currentData; }
};

// === FUNZIONI GLOBALI WRAPPER ===
extern HWT906Handler hwt906;

// API esistente
bool initHWT906();
void updateHWT906();
float getHWT906Pitch();
float getHWT906Yaw();
float getHWT906Roll();
bool isHWT906Ready();
bool isHWT906Present();
void calibrateHWT906();
void zeroHWT906Angles();

float getHWT906PitchRaw();
float getHWT906YawRaw();
float getHWT906RollRaw();

// Dual-mode quaternions
float getHWT906QuaternionW();
float getHWT906QuaternionX();
float getHWT906QuaternionY();
float getHWT906QuaternionZ();

float getHWT906PitchQ();
float getHWT906YawQ();
float getHWT906RollQ();

float getHWT906PitchQFiltered();
float getHWT906YawQFiltered();
float getHWT906RollQFiltered();

bool isHWT906GimbalLockDetected();

// === STEP CAL: FUNZIONI GLOBALI CALIBRAZIONE ===
bool startHWT906GyroCalibration();
bool startHWT906AccelCalibration();
bool startHWT906MagCalibration();

bool isHWT906GyroCalibrationInProgress();
bool isHWT906AccelCalibrationInProgress();
bool isHWT906MagCalibrationInProgress();

float getHWT906GyroCalibrationProgress();
float getHWT906AccelCalibrationProgress();
float getHWT906MagCalibrationProgress();

void finishHWT906GyroCalibration();
void finishHWT906AccelCalibration();
void finishHWT906MagCalibration();

// Compatibility (mantieni per leaf_actions.cpp)
void startMagCalibration();
bool isMagCalibrationInProgress();
float getMagCalibrationProgress();
void finishMagCalibration();

#endif // HWT906_HANDLER_H
