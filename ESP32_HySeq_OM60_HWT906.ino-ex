// === ESP32S3_TouchMenu_FreeRTOS.ino ===
// Versione ibrida con FreeRTOS per sincronizzazione sensori

#include "ui_config.h"
#include "display_utils.h"
#include "touch_handler.h"
#include "leaf_actions.h"
#include "imu_handler.h"
#include "radar_handler.h"
#include "service_menu.h"
#include "menu_state.h"

// === NUOVI INCLUDE FREERTOS ===
#include "sensor_tasks.h"
#include "sync_queue.h"
#include "task_config.h"

#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <CSE_CST328.h>

// === Oggetti globali (esistenti) ===
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0 /* rot */, true);
CSE_CST328 touchObj(240, 320, &Wire1);  // Oggetto reale
CSE_CST328 *touch = &touchObj;           // Puntatore globale

// Intestazione 
char* IntestazioneLCD = "HySeq Plus";
char* PiedipaginaLCD = "by Picobarn - M.Hy.V1";

UIConfig ui;

// === NUOVE VARIABILI GLOBALI ===
static SensorData latestSensorData = {0};
static bool sensorsReady = false;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== HySeq Plus FreeRTOS ===");
    
    // === INIT HARDWARE (esistente) ===
    pinMode(LCD_BL, OUTPUT); 
    digitalWrite(LCD_BL, HIGH);
    Wire.begin(11, 10);       // I2C sensori
    Wire1.begin(TP_SDA, TP_SCL); // I2C touch separato

    // Display init
    gfx->begin();
    gfx->fillScreen(ui.bgColor);

    // Touch init
    if (!touch->begin()) {
        Serial.println("❌ Touch CST328 non rilevato!");
        gfx->setCursor(10, 150);
        gfx->setTextColor(RED);
        gfx->println("Touch Error");
    }

    // === INIT SENSORI (modificato con retry) ===
    bool imuOk = false;
    bool radarOk = false;
    
    for (int retry = 0; retry < 3 && (!imuOk || !radarOk); retry++) {
        if (!imuOk) {
            imuOk = initIMU();
            if (!imuOk) {
                Serial.printf("❌ IMU Init Failed (attempt %d/3)\n", retry + 1);
                delay(100);
            }
        }
        
        if (!radarOk) {
            radarOk = initRadar();
            if (!radarOk) {
                Serial.printf("❌ Radar Init Failed (attempt %d/3)\n", retry + 1);
                delay(100);
            }
        }
    }
    
    if (!imuOk) {
        gfx->setCursor(10, 170);
        gfx->setTextColor(RED);
        gfx->println("IMU Failed!");
    }
    
    if (!radarOk) {
        gfx->setCursor(10, 190);
        gfx->setTextColor(RED);
        gfx->println("Radar Failed!");
    }
    
    sensorsReady = imuOk && radarOk;
    
    // === INIT FREERTOS (nuovo) ===
    if (sensorsReady) {
        if (initSensorTasks()) {
            Serial.println("✅ FreeRTOS tasks started");
            gfx->setCursor(10, 210);
            gfx->setTextColor(GREEN);
            gfx->println("Tasks OK");
        } else {
            Serial.println("❌ FreeRTOS init failed");
            gfx->setCursor(10, 210);
            gfx->setTextColor(RED);
            gfx->println("Tasks Failed!");
        }
    }
    
    // Configurazione UI
    ui.topLine = IntestazioneLCD;
    ui.botLine = PiedipaginaLCD;

    // Mostra menu principale dopo 2 secondi
    delay(2000);
    drawMainMenu(gfx, ui);
}

void loop() {
    // === GESTIONE TOUCH (esistente) ===
    handleTouch(gfx, ui);
    
    // === AGGIORNAMENTO DATI SENSORI (nuovo) ===
    if (sensorsReady) {
        // Ottieni ultimi dati sincronizzati (non bloccante)
        if (getLatestSensorData(latestSensorData)) {
            // I dati sono disponibili per essere usati
            // da displayLiveData() o altre funzioni
            
            // Debug su seriale (opzionale)
            static uint32_t lastDebugPrint = 0;
            if (millis() - lastDebugPrint > 1000) {
                Serial.printf("📊 Sync: %dms | Dist: %.0fmm | Pitch: %.1f° | Yaw: %.1f°\n",
                    latestSensorData.sync_delta_ms,
                    latestSensorData.filtered_distance_mm,
                    latestSensorData.pitch_deg,
                    latestSensorData.yaw_deg
                );
                
                // Statistiche task
                TaskStats stats;
                getSensorTaskStats(stats);
                Serial.printf("📈 Stats: %d samples, %.1f%% sync OK, avg sync: %.1fms\n",
                    stats.samples_acquired,
                    (stats.sync_success * 100.0) / stats.samples_acquired,
                    stats.avg_sync_delta_ms
                );
                
                lastDebugPrint = millis();
            }
        }
    }
    
    // === GESTIONE STATI SPECIALI ===
    // Se siamo nel menu live data, aggiorna display
    if (getCurrentMenuState() == DISPLAY_LIVE_DATA) {
        updateLiveDataDisplay();
    }
    
    // Delay ridotto perché i sensori girano in parallelo
    delay(50);
}

// === NUOVA FUNZIONE PER AGGIORNARE DISPLAY LIVE ===
void updateLiveDataDisplay() {
    static uint32_t lastUpdate = 0;
    
    // Aggiorna a 10Hz per non sovraccaricare
    if (millis() - lastUpdate < 100) return;
    lastUpdate = millis();
    
    if (!sensorsReady) {
        gfx->setCursor(50, 150);
        gfx->setTextColor(RED, ui.bgColor);
        gfx->println("Sensors Not Ready");
        return;
    }
    
    // Usa mutex per proteggere display
    if (TAKE_MUTEX(displayMutex, MS_TO_TICKS(10))) {
        // Distanza
        gfx->setCursor(40, 110);
        gfx->setTextSize(3);
        gfx->setTextColor(YELLOW, RGB565_BLUE);
        gfx->printf("%5.0f mm ", latestSensorData.filtered_distance_mm);
        
        // Pitch
        gfx->setCursor(20, 180);
        gfx->printf("%6.1f deg ", latestSensorData.pitch_deg);
        
        // Yaw
        gfx->setCursor(20, 250);
        gfx->printf("%6.1f deg ", latestSensorData.yaw_deg);
        
        // Indicatore sincronizzazione
        uint16_t syncColor = isSyncValid(latestSensorData) ? GREEN : RED;
        gfx->fillCircle(220, 20, 5, syncColor);
        
        GIVE_MUTEX(displayMutex);
    }
}

// === FUNZIONI HELPER PER MENU ===
// Da aggiungere in leaf_actions.cpp
void displayLiveData() {
    // Questa funzione viene chiamata quando si entra nel menu live
    // Il display verrà aggiornato automaticamente da updateLiveDataDisplay()
    
    // Disegna layout iniziale
    gfx->fillScreen(RGB565_BLUE);
    
    // Header
    gfx->setCursor(40, 20);
    gfx->setTextSize(3);
    gfx->setTextColor(YELLOW);
    gfx->println("Live Data");
    
    // Labels
    gfx->setTextSize(2);
    gfx->setCursor(40, 90);
    gfx->println("Distance");
    gfx->setCursor(40, 150);
    gfx->println("Pitch");
    gfx->setCursor(40, 220);
    gfx->println("Yaw");
    
    // Footer con stato sync
    gfx->setCursor(10, 300);
    gfx->setTextSize(1);
    gfx->println("Sync:");
}

// === GESTIONE CALIBRAZIONE IN BACKGROUND ===
void startCalibration() {
    if (sensorsReady) {
        startBackgroundCalibration();
        
        // Mostra schermata progresso
        gfx->fillScreen(BLACK);
        gfx->setCursor(40, 100);
        gfx->setTextColor(WHITE);
        gfx->println("Calibrating...");
        
        // Il progresso verrà aggiornato in loop()
    }
}

// Helper per mostrare progresso calibrazione
void updateCalibrationProgress() {
    if (isCalibrationComplete()) return;
    
    float progress = getCalibrationProgress();
    
    // Barra progresso
    int barWidth = (int)(200 * progress);
    gfx->fillRect(20, 150, barWidth, 20, GREEN);
    gfx->drawRect(20, 150, 200, 20, WHITE);
    
    // Percentuale
    gfx->setCursor(90, 180);
    gfx->printf("%.0f%%", progress * 100);
}
