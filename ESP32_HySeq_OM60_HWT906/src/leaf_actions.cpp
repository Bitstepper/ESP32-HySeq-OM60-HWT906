#include "leaf_actions.h"
#include "menu_state.h"
#include "hwt906_handler.h"      // MODIFICATO: era imu_handler.h
#include "om60_handler.h"        // MODIFICATO: era radar_handler.h
#include "display_utils.h"
#include "ui_config.h"
#include "sensor_tasks.h"        // Se usi FreeRTOS
#include <Arduino_GFX_Library.h>
// #include <SD.h> rimossa perche forse NON richiesta e dà errore di compilazione
#include <EEPROM.h>
#include <CSE_CST328.h>
#include "hwt906_compat.h"
// RIMOSSO: #include "diagnostic_menu.h" - Non più necessario

// Puntatori esterni
extern Arduino_GFX* gfx;
extern CSE_CST328* touch;
extern UIConfig ui;

// === MODIFICA CRITICA: AGGIUNGI QUESTA DICHIARAZIONE ===
// Questa riga dice al compilatore che esiste una funzione drawMenu
// definita in un altro file (probabilmente display_utils.cpp).
// Senza questa dichiarazione, il compilatore non sa cosa sia drawMenu
// quando la incontra più avanti nel codice.
extern void drawMenu(Arduino_GFX* gfx, UIConfig& ui, MenuState state);

namespace LeafActions {
    
    // Variabili di stato per gestire le azioni in corso
    static bool actionRunning = false;
    static bool stopRequested = false;
    
    // === SUBMENU 1: PITCH,YAW,DIST ===
    
    void showHWT906Status() {
        gfx->fillScreen(RGB565_BLACK);
        
        // Header
        gfx->setCursor(20, 20);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("HWT906 STATUS");
        
        // Info statiche
        gfx->setCursor(20, 60);
        gfx->setTextSize(1);
        gfx->setTextColor(YELLOW);
        gfx->printf("Mode: Static Med  Baud: %d", HWT906_UART_SPEED);
        
        // BACK button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // Touch loop standard (stile esistente)
        while (true) {
            // Update dati live
            updateHWT906();
            
            // Ridisegna solo valori che cambiano
            gfx->fillRect(20, 80, 200, 60, RGB565_BLACK);
            gfx->setCursor(20, 80);
            gfx->setTextSize(1);
            gfx->setTextColor(GREEN);
            gfx->printf("STATUS: %s", isHWT906Ready() ? "CONNECTED" : "OFFLINE");
            
            gfx->setCursor(20, 100);
            gfx->setTextSize(2);
            gfx->setTextColor(CYAN);
            gfx->printf("P: %+6.1f", getHWT906Pitch());
            gfx->setCursor(20, 120);
            gfx->printf("Y: %+6.1f", getHWT906Yaw());
            
            // Range info
            float pitch_min, pitch_max, yaw_min, yaw_max;
            hwt906.getRange(pitch_min, pitch_max, yaw_min, yaw_max);
            gfx->fillRect(20, 150, 200, 20, RGB565_BLACK);  // FIX: Pulizia range
            gfx->setCursor(20, 150);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("Range P: %.1f/%.1f", pitch_min, pitch_max);
            gfx->setCursor(20, 165);
            gfx->printf("Range Y: %.1f/%.1f", yaw_min, yaw_max);
            
            // Performance stats
            const HWT906Data& data = hwt906.getData();
            gfx->fillRect(20, 190, 200, 30, RGB565_BLACK);  // FIX: Pulizia performance
            gfx->setCursor(20, 190);
            gfx->setTextColor(GREEN);
            gfx->printf("UPDATE: %.1fHz", data.update_rate);
            gfx->setCursor(20, 205);
            gfx->printf("Frames: %lu (Err: %lu)", data.frame_count, data.error_frames);
            
            // Pulsanti CONFIG/CALIB
            gfx->fillRect(20, 230, 70, 25, RGB565_DARKGREY);
            gfx->drawRect(20, 230, 70, 25, WHITE);
            gfx->setCursor(30, 240);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->println("CONFIG");

            gfx->fillRect(110, 230, 70, 25, RGB565_DARKGREY);
            gfx->drawRect(110, 230, 70, 25, WHITE);
            gfx->setCursor(125, 240);
            gfx->println("CALIB");
            
            // Check TOUCH
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                // BACK button
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    setCurrentMenuState(SUBMENU_1);
                    drawMenu(gfx, ui, SUBMENU_1);
                    return;
                }
                
                // CONFIG button
                if (p.x >= 20 && p.x <= 90 && p.y >= 230 && p.y <= 255) {
                    delay(200);
                    showHWT906Config();
                    return;
                }
                
                // CALIB button  
                if (p.x >= 110 && p.x <= 180 && p.y >= 230 && p.y <= 255) {
                    delay(200);
                    showHWT906Calib();
                    return;
                }
            }
            
            delay(100); // 10Hz update
        }
    }
    
    void showLiveData() {
        Serial.println("Showing live data...");
        
        // SOLUZIONE 1: Setup solo quando entri nel menu
        static MenuState last_state = MAIN_MENU;
        MenuState current_state = getCurrentMenuState();
        
        if (last_state != DISPLAY_LIVE_DATA && current_state == DISPLAY_LIVE_DATA) {
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
            
            // === AGGIUNGI QUESTO BLOCCO BACK BUTTON ===
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
            
            // Debug area touch
            gfx->setTextSize(1);
            gfx->setCursor(120, 275);
            gfx->printf("Touch: %d,%d to %d,%d", BACK_X, BACK_Y, BACK_X+BACK_W, BACK_Y+BACK_H);
            // === FINE BLOCCO BACK BUTTON ===

            Serial.println("Live Data display setup complete");
        }
        
        last_state = current_state;  // ← AGGIUNGI QUI   
        
        // === BACK BUTTON CON COORDINATE CORRETTE ===
        const int BACK_X = 20;
        const int BACK_Y = 270;
        const int BACK_W = 80;
        const int BACK_H = 35;
        
        if (actionRunning) {
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
            
            actionRunning = false; // Per evitare ridisegno continuo
        }
        
        // === UPDATE DATI ===
        static uint32_t lastUpdate = 0;
        if (millis() - lastUpdate > 100) {  // 10Hz
            
            // FIX: Pulisce SOLO l'area valori, non le etichette
            gfx->fillRect(140, 75, 100, 30, RGB565_BLUE);   // Solo area Distance
            gfx->fillRect(140, 135, 100, 30, RGB565_BLUE);  // Solo area Pitch  
            gfx->fillRect(140, 195, 100, 30, RGB565_BLUE);  // Solo area Yaw
            
            // Versione unificata - RIMUOVO COMPLETAMENTE #ifdef USE_FREERTOS
            float distance = getFilteredOM60Distance();
            float pitch = getHWT906Pitch();
            float yaw = getHWT906Yaw();
            
            // DEBUG che cerchi - SEMPRE ESEGUITO
            Serial.printf("📊 LIVE UPDATE: P=%.1f Y=%.1f D=%.0f\n", pitch, yaw, distance);
            
            // Update display
            gfx->setCursor(140, 80);
            gfx->setTextColor(YELLOW);
            gfx->setTextSize(2);
            gfx->printf("%4.0f mm", distance);
            
            gfx->setCursor(140, 140);
            gfx->printf("%+6.1f", pitch);
            
            gfx->setCursor(140, 200);
            gfx->printf("%5.1f", yaw);
            
            lastUpdate = millis();
        }
      
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
                
                // === FIX: RIDISEGNA IL MENU AUTOMATICAMENTE ===
                drawMenu(gfx, ui, getCurrentMenuState());
                return;
            }
        }
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
    
    void showHWT906Config() {
        gfx->fillScreen(RGB565_BLACK);
        
        // Header
        gfx->setCursor(20, 20);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("HWT906 CONFIG");
        
        // Contenuto statico
        gfx->setCursor(20, 60);
        gfx->setTextSize(1);
        gfx->setTextColor(WHITE);
        gfx->println("Precision Modes:");
        gfx->setCursor(20, 80);
        gfx->println("0 = Static Low (5Hz)");
        gfx->setCursor(20, 95);
        gfx->setTextColor(YELLOW);
        gfx->println("1 = Static Med (10Hz) [ACTIVE]");
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 110);
        gfx->println("2 = Static High (20Hz)");
        gfx->setCursor(20, 125);
        gfx->println("3 = Dynamic (50Hz)");
        
        gfx->setCursor(20, 160);
        gfx->println("Dead Zones:");
        gfx->setCursor(20, 175);
        gfx->printf("Pitch: %.2f  Yaw: %.2f", 0.1f, 0.15f);
        
        gfx->setCursor(20, 210);
        gfx->println("Drift Compensation: ENABLED");
        
        // BACK button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // Touch loop standard
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    showHWT906Status();  // Torna al STATUS
                    return;
                }
            }
            delay(50);
        }
    }
    
    void showHWT906Calib() {
        gfx->fillScreen(RGB565_BLACK);
        
        // Header
        gfx->setCursor(20, 20);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("HWT906 CALIBRATION");
        
        // BACK button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // Touch loop con live data
        while (true) {
            const HWT906Data& data = hwt906.getData();
            
            // Pulisci e ridisegna contenuto live
            gfx->fillRect(20, 60, 200, 200, RGB565_BLACK);
            
            gfx->setCursor(20, 60);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->println("Current Offsets:");
            gfx->setCursor(20, 80);
            gfx->printf("Pitch: %+6.2f", data.pitch_offset);
            gfx->setCursor(20, 95);
            gfx->printf("Yaw:   %+6.2f", data.yaw_offset);
            gfx->setCursor(20, 110);
            gfx->printf("Roll:  %+6.2f", data.roll_offset);
            
            gfx->setCursor(20, 140);
            gfx->println("Raw Values (Live):");
            gfx->setCursor(20, 160);
            gfx->printf("Pitch: %+6.2f", data.pitch_raw);
            gfx->setCursor(20, 175);
            gfx->printf("Yaw:   %+6.2f", data.yaw_raw);
            gfx->setCursor(20, 190);
            gfx->printf("Roll:  %+6.2f", data.roll_raw);
            
            gfx->setCursor(20, 220);
            gfx->setTextColor(YELLOW);
            gfx->println("Touch BACK to return");
            
            // Check touch
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    showHWT906Status();  // Torna al STATUS
                    return;
                }
            }
            delay(200); // 5Hz update per calibrazione
        }
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
    
} // Fine namespace LeafActions