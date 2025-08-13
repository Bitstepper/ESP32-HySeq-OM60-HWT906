// om60_handler.cpp
#include "om60_handler.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <EEPROM.h>

// === VARIABILI GLOBALI ===
static OM60Data currentData = {0};
static OM60CalibrationData calibration = {0};
static esp_adc_cal_characteristics_t adc_chars;

// Buffer per filtri
static float distance_buffer[OM60_MEDIAN_SIZE] = {0};
static uint8_t buffer_index = 0;
static bool buffer_filled = false;

// Statistiche rumore
static float noise_samples[100];
static uint8_t noise_index = 0;

// Forward declarations
void setDefaultCalibration();
void updateNoiseStatistics(); 
float calculateStability();



// Timer per verifiche non bloccanti
static uint32_t lastUpdateTime = 0;
static const uint32_t UPDATE_INTERVAL = 100;  // 10Hz



// === FORWARD DECLARATIONS ===
void setDefaultCalibration();
void updateNoiseStatistics();
float calculateStability();



// === INIZIALIZZAZIONE ===
bool initOM60() {
    Serial.println("🔧 Inizializzazione OM60...");
    
    // Reset stato
    memset(&currentData, 0, sizeof(currentData));
    currentData.sensor_present = false;  // Inizialmente non presente
    
    // Configura ADC1
    adc1_config_width(OM60_ADC_WIDTH);
    adc1_config_channel_atten(OM60_ADC_CHANNEL, OM60_ADC_ATTEN);
    
    // Caratterizzazione ADC per maggior precisione
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_1, 
        OM60_ADC_ATTEN, 
        OM60_ADC_WIDTH,
        1100,  // Vref default
        &adc_chars
    );
    
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.println("✅ ADC calibrato con Vref eFuse");
    } else {
        Serial.println("⚠️ ADC usa Vref default 1100mV");
    }
    
    // Carica calibrazione se presente
    if (!loadOM60Calibration()) {
        Serial.println("⚠️ Calibrazione OM60 non trovata, uso default");
        setDefaultCalibration();
    }
    
    // Test lettura iniziale per verificare presenza sensore
    uint16_t test_reading = 0;
    for (int i = 0; i < 10; i++) {
        test_reading = adc1_get_raw(OM60_ADC_CHANNEL);
        if (test_reading > 100 && test_reading < 4000) {  // Range ragionevole
            currentData.sensor_present = true;
            break;
        }
        delay(10);
    }
    
    if (currentData.sensor_present) {
        Serial.println("✅ OM60 rilevato");
        // Inizializza con alcune letture
        for (int i = 0; i < 10; i++) {
            updateOM60();
            delay(10);
        }
    } else {
        Serial.println("⚠️ OM60 non rilevato - continuo in modalità simulazione");
    }
    
    return true;  // Non bloccante - ritorna sempre true
}

// === ACQUISIZIONE CON OVERSAMPLING ===
uint16_t readADCOversampled() {
    uint32_t sum = 0;
    uint16_t min_val = 4095;
    uint16_t max_val = 0;
    
    // Burst di letture veloci
    for (int burst = 0; burst < OM60_SAMPLES_BURST; burst++) {
        for (int i = 0; i < OM60_OVERSAMPLING; i++) {
            uint16_t val = adc1_get_raw(OM60_ADC_CHANNEL);
            sum += val;
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;
        }
        delayMicroseconds(100); // Breve pausa tra burst
    }
    
    // Media aritmetica
    uint16_t avg = sum / (OM60_OVERSAMPLING * OM60_SAMPLES_BURST);
    
    // Aggiorna statistiche
    currentData.adc_min = min_val;
    currentData.adc_max = max_val;
    currentData.noise_pp = (max_val - min_val) * calibration.slope / OM60_ADC_MAX * OM60_VOLTAGE_SPAN / OM60_VDIV_RATIO;
    
    return avg;
}

// === CONVERSIONE ADC → DISTANZA ===
float convertADCToDistance(uint16_t adc_value) {
    // 1. ADC → Tensione ADC
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_value, &adc_chars);
    float voltage_adc = voltage_mv / 1000.0f;
    
    // 2. Tensione ADC → Tensione ingresso
    float voltage_input = voltage_adc / OM60_VDIV_RATIO;
    
    // 3. Tensione → Distanza (con calibrazione)
    float distance_mm;
    
    if (calibration.r_squared > 0.95f) {
        // Usa calibrazione precisa
        distance_mm = calibration.slope * voltage_input + calibration.offset;
        
        // Applica correzione non-linearità se disponibile
        int index = (distance_mm - OM60_RANGE_MIN) / 100;
        if (index >= 0 && index < 10) {
            distance_mm += calibration.linearity_error[index];
        }
    } else {
        // Fallback lineare semplice
        distance_mm = OM60_RANGE_MIN + (voltage_input / OM60_VOLTAGE_SPAN) * OM60_DISTANCE_SPAN;
    }
    
    // Aggiorna dati
    currentData.voltage_adc = voltage_adc;
    currentData.voltage_input = voltage_input;
    currentData.distance_raw_mm = distance_mm;
    
    // Limita al range valido
    currentData.in_range = (distance_mm >= OM60_RANGE_MIN && distance_mm <= OM60_RANGE_MAX);
    distance_mm = constrain(distance_mm, OM60_RANGE_MIN, OM60_RANGE_MAX);
    
    return distance_mm;
}

// === FILTRO MEDIANO ===
float applyMedianFilter(float new_value) {
    distance_buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % OM60_MEDIAN_SIZE;
    
    if (!buffer_filled && buffer_index == 0) {
        buffer_filled = true;
    }
    
    if (!buffer_filled) {
        return new_value; // Non abbastanza campioni
    }
    
    // Copia e ordina buffer
    float sorted[OM60_MEDIAN_SIZE];
    memcpy(sorted, distance_buffer, sizeof(sorted));
    
    // Bubble sort semplice (ok per 5 elementi)
    for (int i = 0; i < OM60_MEDIAN_SIZE - 1; i++) {
        for (int j = 0; j < OM60_MEDIAN_SIZE - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    // Ritorna mediano
    return sorted[OM60_MEDIAN_SIZE / 2];
}

// === UPDATE PRINCIPALE ===
void updateOM60() {
    // Check timing per 10Hz
    uint32_t now = millis();
    if (now - lastUpdateTime < UPDATE_INTERVAL) {
        return;
    }
    lastUpdateTime = now;
    
    // Timestamp
    currentData.timestamp = now;
    currentData.sample_count++;
    
    if (currentData.sensor_present) {
        // Lettura ADC con oversampling
        currentData.adc_raw = readADCOversampled();
        
        // Conversione a distanza
        float distance_raw = convertADCToDistance(currentData.adc_raw);
        
        // Filtro mediano
        float distance_median = applyMedianFilter(distance_raw);
        
        // Filtro EMA finale
        if (currentData.valid) {
            currentData.distance_mm = currentData.distance_mm * (1 - OM60_EMA_ALPHA) + 
                                      distance_median * OM60_EMA_ALPHA;
        } else {
            currentData.distance_mm = distance_median;
            currentData.valid = true;
        }
    } else {
        // Modalità simulazione se sensore non presente
        static float sim_distance = 500.0f;
        static float sim_direction = 1.0f;
        
        sim_distance += sim_direction * 5.0f;
        if (sim_distance > 800.0f || sim_distance < 300.0f) {
            sim_direction *= -1.0f;
        }
        
        currentData.distance_mm = sim_distance;
        currentData.distance_raw_mm = sim_distance;
        currentData.valid = true;
        currentData.in_range = true;
    }
    
    // Calcola rumore RMS
    updateNoiseStatistics();
    
    // Calcola stabilità
    currentData.stability_score = calculateStability();
}

// === STATISTICHE RUMORE ===
void updateNoiseStatistics() {
    if (!currentData.sensor_present) {
        currentData.noise_rms = 0.0f;
        return;
    }
    
    noise_samples[noise_index] = currentData.distance_raw_mm;
    noise_index = (noise_index + 1) % 100;
    
    // Calcola media
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += noise_samples[i];
    }
    float mean = sum / 100;
    
    // Calcola RMS
    float sum_sq = 0;
    for (int i = 0; i < 100; i++) {
        float diff = noise_samples[i] - mean;
        sum_sq += diff * diff;
    }
    currentData.noise_rms = sqrt(sum_sq / 100);
}

// === CALIBRAZIONE ===
bool calibrateOM60Point(uint8_t index, float reference_mm) {
    if (index >= 5 || !currentData.sensor_present) return false;
    
    Serial.printf("📏 Calibrazione punto %d: %.1fmm\n", index, reference_mm);
    
    // Acquisci 100 campioni stabili
    uint32_t adc_sum = 0;
    float voltage_sum = 0;
    
    for (int i = 0; i < 100; i++) {
        uint16_t adc = readADCOversampled();
        uint32_t mv = esp_adc_cal_raw_to_voltage(adc, &adc_chars);
        adc_sum += adc;
        voltage_sum += (mv / 1000.0f) / OM60_VDIV_RATIO;
        delay(10);
    }
    
    calibration.points[index].reference_mm = reference_mm;
    calibration.points[index].adc_raw = adc_sum / 100;
    calibration.points[index].voltage_measured = voltage_sum / 100;
    calibration.points[index].valid = true;
    
    Serial.printf("✅ Punto %d: ADC=%d, V=%.3f\n", 
                  index, calibration.points[index].adc_raw,
                  calibration.points[index].voltage_measured);
    
    return true;
}

bool calculateOM60Calibration() {
    // Conta punti validi
    int n = 0;
    for (int i = 0; i < 5; i++) {
        if (calibration.points[i].valid) n++;
    }
    
    if (n < 2) {
        Serial.println("❌ Servono almeno 2 punti per calibrazione");
        return false;
    }
    
    // Regressione lineare: distanza = slope * voltage + offset
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    
    for (int i = 0; i < 5; i++) {
        if (!calibration.points[i].valid) continue;
        
        float x = calibration.points[i].voltage_measured;
        float y = calibration.points[i].reference_mm;
        
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    
    calibration.slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    calibration.offset = (sum_y - calibration.slope * sum_x) / n;
    
    // Calcola R²
    float y_mean = sum_y / n;
    float ss_tot = 0, ss_res = 0;
    
    for (int i = 0; i < 5; i++) {
        if (!calibration.points[i].valid) continue;
        
        float y_actual = calibration.points[i].reference_mm;
        float y_pred = calibration.slope * calibration.points[i].voltage_measured + calibration.offset;
        
        ss_tot += pow(y_actual - y_mean, 2);
        ss_res += pow(y_actual - y_pred, 2);
        
        // Salva errore per correzione non-linearità
        int index = (y_actual - OM60_RANGE_MIN) / 100;
        if (index >= 0 && index < 10) {
            calibration.linearity_error[index] = y_actual - y_pred;
        }
    }
    
    calibration.r_squared = 1 - (ss_res / ss_tot);
    
    Serial.printf("✅ Calibrazione completata!\n");
    Serial.printf("   Slope: %.2f mm/V\n", calibration.slope);
    Serial.printf("   Offset: %.2f mm\n", calibration.offset);
    Serial.printf("   R²: %.4f\n", calibration.r_squared);
    
    return true;
}

// === API COMPATIBILITÀ ===
float getOM60Distance() {
    return currentData.distance_raw_mm;
}

float getFilteredOM60Distance() {
    return currentData.distance_mm;
}

OM60Data getOM60Data() {
    return currentData;
}

bool isOM60Ready() {
    return currentData.valid;
}

bool isOM60Present() {
    return currentData.sensor_present;
}

// === DIAGNOSTICA ===
void printOM60Debug() {
    Serial.println("\n=== OM60 DEBUG ===");
    Serial.printf("Presenza: %s\n", currentData.sensor_present ? "SI" : "NO (SIMULAZIONE)");
    
    if (currentData.sensor_present) {
        Serial.printf("ADC Raw: %d [%d-%d]\n", currentData.adc_raw, 
                      currentData.adc_min, currentData.adc_max);
        Serial.printf("Voltage: ADC=%.3fV, Input=%.3fV\n", 
                      currentData.voltage_adc, currentData.voltage_input);
        Serial.printf("Noise: PP=%.3fmm, RMS=%.3fmm\n", 
                      currentData.noise_pp, currentData.noise_rms);
    }
    
    Serial.printf("Distance: Raw=%.2fmm, Filtered=%.2fmm\n", 
                  currentData.distance_raw_mm, currentData.distance_mm);
    Serial.printf("Quality: Stability=%.1f%%, In Range=%s\n", 
                  currentData.stability_score, currentData.in_range ? "YES" : "NO");
    Serial.printf("Samples: %lu @ %.1fHz\n", 
                  currentData.sample_count, 
                  1000.0f * currentData.sample_count / millis());
}

float getOM60Resolution() {
    // Risoluzione effettiva basata su rumore
    if (!currentData.sensor_present) return 1.0f;  // 1mm in simulazione
    return max(0.05f, currentData.noise_rms * 3); // 3-sigma
}

// === FUNZIONI HELPER ===
void setDefaultCalibration() {
    calibration.slope = OM60_DISTANCE_SPAN / OM60_VOLTAGE_SPAN;
    calibration.offset = OM60_RANGE_MIN;
    calibration.r_squared = 0.0;
    
    for (int i = 0; i < 10; i++) {
        calibration.linearity_error[i] = 0;
    }
}

float calculateStability() {
    if (currentData.noise_rms < 0.05) return 100.0;
    if (currentData.noise_rms < 0.1) return 90.0;
    if (currentData.noise_rms < 0.2) return 75.0;
    if (currentData.noise_rms < 0.5) return 50.0;
    return 25.0;
}

// === EEPROM ===
void saveOM60Calibration() {
    EEPROM.begin(512);
    int addr = 100; // Offset per OM60
    
    EEPROM.put(addr, calibration);
    EEPROM.write(addr + sizeof(calibration), 0xAA); // Marker
    
    EEPROM.commit();
    Serial.println("✅ Calibrazione OM60 salvata");
}

bool loadOM60Calibration() {
    EEPROM.begin(512);
    int addr = 100;
    
    if (EEPROM.read(addr + sizeof(calibration)) != 0xAA) {
        return false;
    }
    
    EEPROM.get(addr, calibration);
    Serial.println("✅ Calibrazione OM60 caricata");
    return true;
}

// === CONFIGURAZIONE ===
void setOM60FilterMode(uint8_t mode) {
    // TODO: Implementare diversi modi di filtraggio
    Serial.printf("Filter mode richiesto: %d\n", mode);
}

void setOM60UpdateRate(uint16_t ms) {
    // TODO: Implementare update rate variabile
    Serial.printf("Update rate richiesto: %d ms\n", ms);
}

void runOM60SelfTest() {
    Serial.println("\n=== OM60 SELF TEST ===");
    
    if (!currentData.sensor_present) {
        Serial.println("❌ Sensore non presente - in modalità simulazione");
        return;
    }
    
    // Test range ADC
    Serial.println("Test 1: Range ADC");
    for (int i = 0; i < 10; i++) {
        uint16_t val = adc1_get_raw(OM60_ADC_CHANNEL);
        Serial.printf("  ADC[%d]: %d\n", i, val);
        delay(100);
    }
    
    // Test stabilità
    Serial.println("Test 2: Stabilità (1 secondo)");
    float min_dist = 9999, max_dist = 0;
    for (int i = 0; i < 10; i++) {
        updateOM60();
        if (currentData.distance_mm < min_dist) min_dist = currentData.distance_mm;
        if (currentData.distance_mm > max_dist) max_dist = currentData.distance_mm;
        delay(100);
    }
    Serial.printf("  Range: %.2f - %.2f mm (delta: %.2f mm)\n", 
                  min_dist, max_dist, max_dist - min_dist);
    
    Serial.println("=== TEST COMPLETATO ===\n");
}





// === STUB FUNCTIONS PER COMPATIBILITÀ ===
