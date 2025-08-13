#include "leaf_actions.h"
#include "menu_state.h"
#include "hwt906_handler.h"      // MODIFICATO: era imu_handler.h
#include "om60_handler.h"        // MODIFICATO: era radar_handler.h
#include "display_utils.h"
#include "ui_config.h"
#include "sensor_tasks.h"        // Se usi FreeRTOS
#include <Arduino_GFX_Library.h>
#include <SD.h>
#include <EEPROM.h>
#include <CSE_CST328.h>

// Puntatori esterni
extern Arduino_GFX* gfx;
extern CSE_CST328* touch;
extern UIConfig ui;

// === MODIFICA CRITICA: AGGIUNGI QUESTA DICHIARAZIONE ===
// Questa riga dice al compilatore che esiste una funzione drawMenu
// definita in un altro file (probabilmente display_utils.cpp).
// Senza questa dichiarazione, il compilatore non sa cosa sia drawMenu
// quando la incontra piÃ¹ avanti nel codice.
extern void drawMenu(Arduino_GFX* gfx, UIConfig& ui, MenuState state);

namespace LeafActions {
    
    // Variabili di stato per gestire le azioni in corso
    static bool actionRunning = false;
    static bool stopRequested = false;
    
    // === SUBMENU 1: PITCH,YAW,DIST ===
    
    void startDataAcquisition() {
        Serial.println("Starting data acquisition...");
        actionRunning = true;
        
        // === MODIFICA: AGGIUNGI DISPLAY INIZIALE E DEBUG ===
        gfx->fillScreen(BLACK);
        gfx->setCursor(50, 150);
        gfx->setTextColor(WHITE);
        gfx->println("Starting...");
        
        // AGGIUNGI QUESTE RIGHE DI DEBUG
        Serial.println("DEBUG: Start Data Acquisition");
        Serial.printf("HWT906 present: %s\n", isHWT906Present() ? "YES" : "NO");
        Serial.printf("OM60 present: %s\n", isOM60Present() ? "YES" : "NO");
        
        // === MODIFICA: CONTROLLO SENSORI CON DEMO MODE ===
        if (!isHWT906Present() && !isOM60Present()) {
            gfx->setTextColor(ORANGE);
            gfx->setCursor(30, 170);
            gfx->println("DEMO MODE - No sensors");
            gfx->setCursor(20, 190);
            gfx->println("Proceeding with simulation");
            delay(2000);
            // NON fare return - continua in modalità demo
        } else if (!isHWT906Present() || !isOM60Present()) {
            gfx->setTextColor(YELLOW);
            gfx->setCursor(30, 170);
            gfx->println("Partial sensor mode");
            delay(1500);
        }
        
        // === CONTINUA CON ACQUISIZIONE (REALE O SIMULATA) ===
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(20, 50);
        gfx->println("ACQUIRING DATA");
        
        // Mostra cosa stiamo acquisendo
        gfx->setTextSize(1);
        gfx->setCursor(20, 100);
        if (isHWT906Present()) {
            gfx->setTextColor(GREEN);
            gfx->println("✓ HWT906: Real data");
        } else {
            gfx->setTextColor(ORANGE);
            gfx->println("O HWT906: Simulated");
        }
        
        gfx->setCursor(20, 120);
        if (isOM60Present()) {
            gfx->setTextColor(GREEN);
            gfx->println("✓ OM60: Real data");
        } else {
            gfx->setTextColor(ORANGE);
            gfx->println("O OM60: Simulated");
        }
        
        // Simulazione acquisizione con progress bar
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 160);
        gfx->println("Progress:");
        
        for (int i = 0; i <= 100; i += 10) {
            // Progress bar
            gfx->fillRect(20, 180, i * 2, 20, GREEN);
            gfx->drawRect(20, 180, 200, 20, WHITE);
            
            // Percentuale
            gfx->fillRect(90, 205, 60, 20, BLACK);
            gfx->setCursor(95, 208);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("%d%%", i);
            
            // TODO: Qui acquisire dati reali
            // if (isHWT906Present()) { ... }
            // if (isOM60Present()) { ... }
            
            delay(200);  // Simulazione tempo acquisizione
        }
        
        gfx->setCursor(60, 240);
        gfx->setTextColor(GREEN);
        gfx->setTextSize(2);
        gfx->println("COMPLETE!");
        
        delay(2000);  // Mostra completamento
        
        actionRunning = false;
        Serial.println("Acquisition complete");
        
        // Torna al menu principale
        setCurrentMenuState(MAIN_MENU);
        drawMenu(gfx, ui, MAIN_MENU);
    }
    
    void showLiveData() {
        Serial.println("Showing live data...");
        actionRunning = true;
        stopRequested = false;
        
        // Setup display
        gfx->fillScreen(RGB565_BLUE);
        
        // Header
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(3);
        gfx->setCursor(50, 20);
        gfx->println("LIVE DATA");
        
        // Labels
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 80);
        gfx->println("Distance:");
        gfx->setCursor(20, 140);
        gfx->println("Pitch:");
        gfx->setCursor(20, 200);
        gfx->println("Yaw:");
        
        // === BACK BUTTON CON COORDINATE CORRETTE ===
        const int BACK_X = 20;
        const int BACK_Y = 270;
        const int BACK_W = 80;
        const int BACK_H = 35;
        
        gfx->fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ORANGE);
        gfx->drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, WHITE);
        gfx->setCursor(BACK_X + 15, BACK_Y + 10);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // === DEBUG: Mostra area touch ===
        gfx->setTextSize(1);
        gfx->setCursor(120, 275);
        gfx->printf("Touch: %d,%d to %d,%d", BACK_X, BACK_Y, BACK_X+BACK_W, BACK_Y+BACK_H);
        
        // Live update loop con gestione touch integrata
        while (!stopRequested && isInLiveDataMode()) {
            // === UPDATE DATI ===
            #ifdef USE_FREERTOS
            SensorData data;
            if (getLatestSensorData(data)) {
                // Distanza
                gfx->fillRect(140, 75, 80, 30, RGB565_BLUE);
                gfx->setCursor(140, 80);
                gfx->setTextColor(YELLOW);
                gfx->setTextSize(2);
                gfx->printf("%4.0f mm", data.filtered_distance_mm);
                
                // Pitch
                gfx->fillRect(140, 135, 80, 30, RGB565_BLUE);
                gfx->setCursor(140, 140);
                gfx->printf("%+6.1f", data.pitch);
                
                // Yaw
                gfx->fillRect(140, 195, 80, 30, RGB565_BLUE);
                gfx->setCursor(140, 200);
                gfx->printf("%5.1f", data.yaw);
            }
            #else
            // Versione senza FreeRTOS
            float distance = getFilteredOM60Distance();  // MODIFICATO: era getFilteredRadarDistance()
            float pitch = getHWT906Pitch();              // MODIFICATO: era getIMUPitch()
            float yaw = getHWT906Yaw();                  // MODIFICATO: era getIMUYaw()
            
            // Update display
            gfx->fillRect(140, 75, 80, 30, RGB565_BLUE);
            gfx->setCursor(140, 80);
            gfx->setTextColor(YELLOW);
            gfx->setTextSize(2);
            gfx->printf("%4.0f mm", distance);
            
            gfx->fillRect(140, 135, 80, 30, RGB565_BLUE);
            gfx->setCursor(140, 140);
            gfx->printf("%+6.1f", pitch);
            
            gfx->fillRect(140, 195, 80, 30, RGB565_BLUE);
            gfx->setCursor(140, 200);
            gfx->printf("%5.1f", yaw);
            #endif
            
            // === CHECK TOUCH DIRETTAMENTE QUI ===
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                // Debug: mostra coordinate touch
                gfx->fillRect(120, 290, 100, 20, RGB565_BLUE);
                gfx->setCursor(120, 290);
                gfx->setTextSize(1);
                gfx->setTextColor(WHITE);
                gfx->printf("X:%d Y:%d", p.x, p.y);
                
                // Check BACK button con margine
                if (p.x >= (BACK_X - 10) && p.x <= (BACK_X + BACK_W + 10) && 
                    p.y >= (BACK_Y - 10) && p.y <= (BACK_Y + BACK_H + 10)) {
                    
                    // Feedback visivo
                    gfx->fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, RED);
                    gfx->drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, WHITE);
                    gfx->setCursor(BACK_X + 15, BACK_Y + 10);
                    gfx->setTextSize(2);
                    gfx->setTextColor(WHITE);
                    gfx->println("BACK");
                    
                    delay(200);  // Debounce
                    
                    // Esci dal loop
                    stopRequested = true;
                    setCurrentMenuState(SUBMENU_1);  // Torna al submenu
                    Serial.println("Back pressed - exiting live data");
                }
            }
            
            delay(100);  // 10Hz update
        }
        
        actionRunning = false;
        
        // === FIX: RIDISEGNA IL MENU AUTOMATICAMENTE ===
        // Ora questa chiamata funzionerÃ  perchÃ© abbiamo dichiarato drawMenu sopra
        drawMenu(gfx, ui, getCurrentMenuState());
    }
    
    void showLiveGraph() {
        Serial.println("Showing live graph...");
        
        // === MODIFICA: IMPLEMENTAZIONE DEMO GRAPH ===
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(30, 20);
        gfx->println("LIVE GRAPH");
        
        // Disegna assi
        gfx->drawLine(30, 240, 230, 240, WHITE);  // Asse X
        gfx->drawLine(30, 60, 30, 240, WHITE);    // Asse Y
        
        // Labels
        gfx->setTextSize(1);
        gfx->setCursor(5, 60);
        gfx->print("1000");
        gfx->setCursor(5, 150);
        gfx->print("500");
        gfx->setCursor(5, 235);
        gfx->print("0");
        gfx->setCursor(100, 250);
        gfx->print("Time");
        
        // Back button
        gfx->fillRect(150, 270, 80, 35, ORANGE);
        gfx->drawRect(150, 270, 80, 35, WHITE);
        gfx->setCursor(175, 282);
        gfx->setTextSize(2);
        gfx->println("BACK");
        
        // Demo graph animation
        int x = 31;
        float lastY = 150;
        
        while (!stopRequested) {
            // Genera dati demo
            float distance = isOM60Present() ? getFilteredOM60Distance() : 
                           500.0 + sin(millis() / 1000.0) * 200;
            
            // Scala il valore per il grafico (60-240 pixel per 0-1000mm)
            float y = 240 - (distance / 1000.0 * 180);
            
            // Disegna linea
            if (x > 31) {
                gfx->drawLine(x-1, lastY, x, y, GREEN);
            }
            
            lastY = y;
            x++;
            
            // Reset quando raggiungi il bordo
            if (x > 230) {
                // Pulisci area grafico
                gfx->fillRect(31, 61, 199, 179, BLACK);
                x = 31;
            }
            
            // Check touch per BACK
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 150 && p.x <= 230 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    stopRequested = true;
                    setCurrentMenuState(SUBMENU_1);
                }
            }
            
            delay(50);  // 20Hz update
        }
        
        stopRequested = false;
        drawMenu(gfx, ui, getCurrentMenuState());
    }
    
    void exportCSV() {
        Serial.println("Exporting to CSV...");
        
        // TODO: Implementare export
        // - Aprire file su SD
        // - Scrivere header CSV
        // - Scrivere dati buffer
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(GREEN);
        gfx->setCursor(50, 150);
        gfx->println("Exporting...");
        
        delay(2000);
        
        gfx->setCursor(50, 180);
        gfx->println("Complete!");
        delay(1000);
    }
    
    // === SUBMENU 2: CALIB. IMU ===
    
    void calibrateGyro() {
        Serial.println("Calibrating Gyroscope...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setCursor(30, 100);
        gfx->setTextSize(2);
        gfx->println("GYRO CALIBRATION");
        
        gfx->setCursor(20, 150);
        gfx->setTextSize(1);
        gfx->println("Keep device still for 5 sec");
        
        // Progress bar
        for (int i = 0; i <= 100; i += 20) {
            gfx->fillRect(20, 180, i * 2, 20, GREEN);
            gfx->drawRect(20, 180, 200, 20, WHITE);
            delay(1000);
        }
        
        gfx->setCursor(70, 220);
        gfx->setTextColor(GREEN);
        gfx->println("COMPLETE!");
        
        delay(1500);
        setCurrentMenuState(SUBMENU_2);
        // Ridisegna menu dopo calibrazione
        drawMenu(gfx, ui, SUBMENU_2);
    }
    
    void calibrateAccel() {
        Serial.println("Calibrating Accelerometer...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 100);
        gfx->setTextSize(2);
        gfx->println("ACCEL CALIBRATION");
        
        gfx->setCursor(20, 150);
        gfx->setTextSize(1);
        gfx->println("Place on flat surface");
        
        delay(3000);
        
        gfx->setCursor(70, 220);
        gfx->setTextColor(GREEN);
        gfx->println("COMPLETE!");
        
        delay(1500);
        setCurrentMenuState(SUBMENU_2);
        // Ridisegna menu dopo calibrazione
        drawMenu(gfx, ui, SUBMENU_2);
    }
    
    void calibrateMag() {
        Serial.println("Calibrating Magnetometer...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 80);
        gfx->setTextSize(2);
        gfx->println("MAG CALIBRATION");
        
        gfx->setCursor(20, 130);
        gfx->setTextSize(1);
        gfx->println("Rotate device in all");
        gfx->setCursor(20, 150);
        gfx->println("directions for 20 sec");
        
        // Avvia calibrazione HWT906
        startMagCalibration();
        
        // Progress con Cancel button
        gfx->fillRect(70, 250, 100, 40, RED);
        gfx->drawRect(70, 250, 100, 40, WHITE);
        gfx->setCursor(95, 265);
        gfx->setTextSize(2);
        gfx->println("CANCEL");
        
        // Mostra progresso
        while (isMagCalibrationInProgress()) {
            float progress = getMagCalibrationProgress();
            
            // Progress bar
            int barWidth = (int)(200 * progress);
            gfx->fillRect(20, 200, barWidth, 20, GREEN);
            gfx->drawRect(20, 200, 200, 20, WHITE);
            
            // Percentuale
            gfx->fillRect(90, 175, 60, 20, BLACK);
            gfx->setCursor(95, 178);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("%.0f%%", progress * 100);
            
            // Check per CANCEL
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 70 && p.x <= 170 && p.y >= 250 && p.y <= 290) {
                    Serial.println("Calibration cancelled");
                    break;
                }
            }
            
            delay(100);
        }
        
        finishMagCalibration();
        
        gfx->fillRect(20, 240, 200, 60, BLACK);
        gfx->setCursor(70, 250);
        gfx->setTextColor(GREEN);
        gfx->setTextSize(2);
        gfx->println("COMPLETE!");
        
        delay(1500);
        setCurrentMenuState(SUBMENU_2);
        // Ridisegna menu dopo calibrazione
        drawMenu(gfx, ui, SUBMENU_2);
    }
    
    // === SUBMENU 3: SERVICE ===
    
    void changeSettings() {
        Serial.println("Opening settings...");
        
        // Menu impostazioni migliorato
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(50, 20);
        gfx->println("SETTINGS");
        
        // Opzioni
        gfx->setTextSize(1);
        gfx->setCursor(20, 80);
        gfx->println("1. Dead Zones");
        gfx->setCursor(20, 100);
        gfx->println("2. Filter Parameters");
        gfx->setCursor(20, 120);
        gfx->println("3. OM60 Range");           // MODIFICATO: era "Radar Range"
        gfx->setCursor(20, 140);
        gfx->println("4. Sample Rate");
        
        // Back button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // Attendi back
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    break;
                }
            }
            delay(50);
        }
        
        // Ridisegna menu service
        drawMenu(gfx, ui, SUBMENU_3);
    }
    
    void showSystemInfo() {
        Serial.println("System Information");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(20, 20);
        gfx->println("SYSTEM INFO");
        
        gfx->setTextSize(1);
        gfx->setCursor(20, 60);
        gfx->println("HySeq OM60/HWT906 v2.0");    // MODIFICATO: aggiornato nome versione
        
        gfx->setCursor(20, 80);
        gfx->printf("Free Heap: %lu KB", ESP.getFreeHeap() / 1024);
        
        gfx->setCursor(20, 100);
        gfx->printf("CPU Freq: %lu MHz", ESP.getCpuFreqMHz());
        
        gfx->setCursor(20, 120);
        gfx->printf("Flash: %lu MB", ESP.getFlashChipSize() / 1048576);
        
        // Stato sensori
        gfx->setCursor(20, 150);
        gfx->print("HWT906: ");                      // MODIFICATO: era "IMU: "
        gfx->setTextColor(isHWT906Ready() ? GREEN : RED);    // MODIFICATO: era isIMUReady()
        gfx->println(isHWT906Ready() ? "OK" : "ERROR");      // MODIFICATO: era isIMUReady()
        
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 170);
        gfx->print("OM60: ");                        // MODIFICATO: era "Radar: "
        gfx->setTextColor(isOM60Ready() ? GREEN : RED);      // MODIFICATO: era isRadarReady()
        gfx->println(isOM60Ready() ? "OK" : "ERROR");        // MODIFICATO: era isRadarReady()
        
        // Test sensori in tempo reale
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 190);
        gfx->println("--- Live Sensor Data ---");
        
        if (isHWT906Ready()) {                       // MODIFICATO: era isIMUReady()
            gfx->setCursor(20, 210);
            gfx->printf("Pitch: %.1f  Yaw: %.1f", getHWT906Pitch(), getHWT906Yaw());  // MODIFICATO: era getIMUPitch(), getIMUYaw()
        }
        
        if (isOM60Ready()) {                         // MODIFICATO: era isRadarReady()
            gfx->setCursor(20, 230);
            gfx->printf("Distance: %.0f mm", getFilteredOM60Distance());  // MODIFICATO: era getFilteredRadarDistance()
        }
        
        // Statistiche
        #ifdef USE_FREERTOS
        TaskStats stats;
        getSensorTaskStats(stats);
        
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 250);
        gfx->printf("Samples: %d", stats.samples_acquired);
        #endif
        
        // Back button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // Attendi back
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    break;
                }
            }
            delay(50);
        }
        
        // Ridisegna menu service
        drawMenu(gfx, ui, SUBMENU_3);
    }
    
    void factoryReset() {
        Serial.println("Factory Reset!");
        
        // Conferma
        gfx->fillScreen(BLACK);
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        gfx->setCursor(40, 80);
        gfx->println("CONFIRM RESET?");
        
        gfx->setTextColor(WHITE);
        gfx->setTextSize(1);
        gfx->setCursor(30, 120);
        gfx->println("This will erase all data!");
        
        // Pulsanti SI/NO
        gfx->fillRect(40, 180, 60, 40, RED);
        gfx->drawRect(40, 180, 60, 40, WHITE);
        gfx->setCursor(55, 195);
        gfx->setTextSize(2);
        gfx->println("YES");
        
        gfx->fillRect(140, 180, 60, 40, GREEN);
        gfx->drawRect(140, 180, 60, 40, WHITE);
        gfx->setCursor(160, 195);
        gfx->println("NO");
        
        bool confirmed = false;
        
        // Attendi risposta
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                // YES
                if (p.x >= 40 && p.x <= 100 && p.y >= 180 && p.y <= 220) {
                    confirmed = true;
                    delay(200);
                    break;
                }
                
                // NO
                if (p.x >= 140 && p.x <= 200 && p.y >= 180 && p.y <= 220) {
                    confirmed = false;
                    delay(200);
                    break;
                }
            }
            delay(50);
        }
        
        if (confirmed) {
            // Esegui reset
            gfx->fillScreen(BLACK);
            gfx->setTextColor(RED);
            gfx->setTextSize(2);
            gfx->setCursor(50, 120);
            gfx->println("RESETTING...");
            
            // Cancella EEPROM
            EEPROM.begin(512);
            for (int i = 0; i < 512; i++) {
                EEPROM.write(i, 0xFF);
            }
            EEPROM.commit();
            
            // Reset preferenze
            // TODO: Reset altre impostazioni
            
            gfx->setCursor(40, 160);
            gfx->println("COMPLETE!");
            gfx->setCursor(30, 200);
            gfx->println("Rebooting...");
            
            delay(2000);
            ESP.restart();
        } else {
            // Annullato
            gfx->fillScreen(BLACK);
            gfx->setTextColor(GREEN);
            gfx->setCursor(60, 150);
            gfx->println("CANCELLED");
            delay(1000);
            
            // Torna al menu
            drawMenu(gfx, ui, SUBMENU_3);
        }
    }
    
    // === UTILITY ===
    
    void stopCurrentAction() {
        stopRequested = true;
    }
    
    bool isActionRunning() {
        return actionRunning;
    }
}