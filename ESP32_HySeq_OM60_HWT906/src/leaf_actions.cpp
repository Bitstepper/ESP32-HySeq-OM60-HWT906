// leaf_actions.cpp - Fixed Calibration Issues
// FIXES: Gyro progress, Accel touch, Mag countdown
// 🔧 CRITICAL FIXES APPLIED: Raddoppiati tutti i tempi di acquisizione

#include "leaf_actions.h"
#include "menu_state.h"
#include "hwt906_handler.h"
#include "om60_handler.h"
#include "display_utils.h"
#include "ui_config.h"
#include "sensor_tasks.h"
#include <Arduino_GFX_Library.h>
#include <CSE_CST328.h>
#include "hwt906_compat.h"

// Puntatori esterni
extern Arduino_GFX* gfx;
extern CSE_CST328* touch;
extern UIConfig ui;

// Dichiarazione esterna
extern void drawMenu(Arduino_GFX* gfx, UIConfig& ui, MenuState state);

namespace LeafActions {
    
    static bool actionRunning = false;
    static bool stopRequested = false;
    
    // === SUBMENU 1: PITCH,YAW,DIST (unchanged) ===
    
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
        
        while (true) {
            updateHWT906();
            
            // Status principale
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
            gfx->fillRect(20, 150, 200, 20, RGB565_BLACK);
            gfx->setCursor(20, 150);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("Range P: %.1f/%.1f", pitch_min, pitch_max);
            gfx->setCursor(20, 165);
            gfx->printf("Range Y: %.1f/%.1f", yaw_min, yaw_max);
            
            // Performance stats
            const HWT906Data& data = hwt906.getData();
            gfx->fillRect(20, 190, 200, 30, RGB565_BLACK);
            gfx->setCursor(20, 190);
            gfx->setTextColor(GREEN);
            gfx->printf("UPDATE: %.1fHz", data.update_rate);
            gfx->setCursor(20, 205);
            gfx->printf("Frames: %lu (Err: %lu)", data.frame_count, data.error_frames);
            
            // Enhanced status display
            gfx->fillRect(20, 220, 200, 35, RGB565_BLACK);
            gfx->setCursor(20, 220);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("CALIBRATIONS:");
            gfx->setCursor(20, 235);
            gfx->setTextColor(hwt906.isGyroCalibrated() ? GREEN : RED);
            gfx->printf("G:%s ", hwt906.isGyroCalibrated() ? "OK" : "NO");
            gfx->setTextColor(hwt906.isAccelCalibrated() ? GREEN : RED);
            gfx->printf("A:%s ", hwt906.isAccelCalibrated() ? "OK" : "NO");
            gfx->setTextColor(hwt906.isMagCalibrated() ? GREEN : RED);
            gfx->printf("M:%s", hwt906.isMagCalibrated() ? "OK" : "NO");
            
            // Pulsanti CONFIG/CALIB
            gfx->fillRect(20, 250, 70, 25, RGB565_DARKGREY);
            gfx->drawRect(20, 250, 70, 25, WHITE);
            gfx->setCursor(30, 260);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->println("CONFIG");

            gfx->fillRect(110, 250, 70, 25, RGB565_DARKGREY);
            gfx->drawRect(110, 250, 70, 25, WHITE);
            gfx->setCursor(125, 260);
            gfx->println("CALIB");
            
            // Check TOUCH
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    setCurrentMenuState(SUBMENU_1);
                    drawMenu(gfx, ui, SUBMENU_1);
                    return;
                }
                
                if (p.x >= 20 && p.x <= 90 && p.y >= 250 && p.y <= 275) {
                    delay(200);
                    showHWT906Config();
                    return;
                }
                
                if (p.x >= 110 && p.x <= 180 && p.y >= 250 && p.y <= 275) {
                    delay(200);
                    showHWT906Calib();
                    return;
                }
            }
            
            delay(100);
        }
    }
    
    void showLiveData() {
        Serial.println("📊 Showing DUAL-MODE live data (Euler + Quaternions)...");
        actionRunning = true;
        stopRequested = false;
        
        gfx->fillScreen(RGB565_BLUE);
        
        // Header con indicatore stato
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        gfx->setCursor(20, 20);
        gfx->println("LIVE DATA");
        
        // Status indicator (top-right)
        gfx->fillRect(200, 20, 40, 20, GREEN);
        gfx->setCursor(205, 25);
        gfx->setTextSize(1);
        gfx->setTextColor(WHITE);
        gfx->println("DUAL");
        
        // Layout dual-mode esteso
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->setCursor(10, 60);
        gfx->println("Distance:");
        
        // Due colonne per confronto
        gfx->setTextSize(1);
        gfx->setTextColor(WHITE);
        gfx->setCursor(10, 90);
        gfx->println("EULER (from IMU):");
        gfx->setCursor(130, 90);
        gfx->println("EULER (from Q):");
        
        // Labels statici pitch
        gfx->setCursor(10, 110);
        gfx->println("Pitch:");
        gfx->setCursor(10, 125);
        gfx->setTextColor(LIGHTGREY);
        gfx->println("PitchX:");
        gfx->setTextColor(WHITE);
        gfx->setCursor(130, 110);
        gfx->println("Pitch_Q:");
        gfx->setCursor(130, 125);
        gfx->setTextColor(LIGHTGREY);
        gfx->println("Pitch_QF:");
        
        // Labels statici yaw e roll
        gfx->setTextColor(WHITE);
        gfx->setCursor(10, 150);
        gfx->println("Yaw:");
        gfx->setCursor(10, 165);
        gfx->setTextColor(LIGHTGREY);
        gfx->println("YawX:");
        gfx->setTextColor(WHITE);
        gfx->setCursor(130, 150);
        gfx->println("Yaw_Q:");
        gfx->setCursor(130, 165);
        gfx->setTextColor(LIGHTGREY);
        gfx->println("Yaw_QF:");
        
        gfx->setTextColor(WHITE);
        gfx->setCursor(10, 190);
        gfx->println("Roll:");
        gfx->setCursor(10, 205);
        gfx->setTextColor(LIGHTGREY);
        gfx->println("RollX:");
        gfx->setTextColor(WHITE);
        gfx->setCursor(130, 190);
        gfx->println("Roll_Q:");
        gfx->setCursor(130, 205);
        gfx->setTextColor(LIGHTGREY);
        gfx->println("Roll_QF:");
        
        // Quaternioni raw
        gfx->setTextColor(YELLOW);
        gfx->setCursor(10, 230);
        gfx->println("QUATERNIONS (raw):");
        
        // BACK button
        const int BACK_X = 80, BACK_Y = 270, BACK_W = 80, BACK_H = 35;
        gfx->fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ORANGE);
        gfx->drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, WHITE);
        gfx->setCursor(BACK_X + 15, BACK_Y + 10);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        // Main loop dual-mode
        while (true) {
            updateHWT906();
            
            static uint32_t lastUpdate = 0;
            if (millis() - lastUpdate > 100) {
                
                // Distance
                float distance = 850.0f; // getFilteredOM60Distance() quando disponibile
                gfx->fillRect(140, 60, 100, 20, RGB565_BLUE);
                gfx->setCursor(140, 60);
                gfx->setTextSize(2);
                gfx->setTextColor(YELLOW);
                gfx->printf("%4.0f mm", distance);
                
                // Colonna sinistra: Euler tradizionali
                gfx->fillRect(60, 110, 60, 12, RGB565_BLUE);
                gfx->setCursor(60, 110);
                gfx->setTextSize(1);
                gfx->setTextColor(CYAN);
                gfx->printf("%+6.2f", getHWT906Pitch());
                
                gfx->fillRect(60, 125, 60, 12, RGB565_BLUE);
                gfx->setCursor(60, 125);
                gfx->setTextColor(LIGHTGREY);
                gfx->printf("%+6.2f", getHWT906PitchRaw());
                
                gfx->fillRect(60, 150, 60, 12, RGB565_BLUE);
                gfx->setCursor(60, 150);
                gfx->setTextColor(CYAN);
                gfx->printf("%+6.2f", getHWT906Yaw());
                
                gfx->fillRect(60, 165, 60, 12, RGB565_BLUE);
                gfx->setCursor(60, 165);
                gfx->setTextColor(LIGHTGREY);
                gfx->printf("%+6.2f", getHWT906YawRaw());
                
                gfx->fillRect(60, 190, 60, 12, RGB565_BLUE);
                gfx->setCursor(60, 190);
                gfx->setTextColor(CYAN);
                gfx->printf("%+6.2f", getHWT906Roll());
                
                gfx->fillRect(60, 205, 60, 12, RGB565_BLUE);
                gfx->setCursor(60, 205);
                gfx->setTextColor(LIGHTGREY);
                gfx->printf("%+6.2f", getHWT906RollRaw());
                
                // Colonna destra: Euler da quaternioni
                gfx->fillRect(180, 110, 60, 12, RGB565_BLUE);
                gfx->setCursor(180, 110);
                gfx->setTextColor(ORANGE);
                gfx->printf("%+6.2f", getHWT906PitchQ());
                
                gfx->fillRect(180, 125, 60, 12, RGB565_BLUE);
                gfx->setCursor(180, 125);
                gfx->setTextColor(MAGENTA);
                gfx->printf("%+6.2f", getHWT906PitchQFiltered());
                
                gfx->fillRect(180, 150, 60, 12, RGB565_BLUE);
                gfx->setCursor(180, 150);
                gfx->setTextColor(ORANGE);
                gfx->printf("%+6.2f", getHWT906YawQ());
                
                gfx->fillRect(180, 165, 60, 12, RGB565_BLUE);
                gfx->setCursor(180, 165);
                gfx->setTextColor(MAGENTA);
                gfx->printf("%+6.2f", getHWT906YawQFiltered());
                
                gfx->fillRect(180, 190, 60, 12, RGB565_BLUE);
                gfx->setCursor(180, 190);
                gfx->setTextColor(ORANGE);
                gfx->printf("%+6.2f", getHWT906RollQ());
                
                gfx->fillRect(180, 205, 60, 12, RGB565_BLUE);
                gfx->setCursor(180, 205);
                gfx->setTextColor(MAGENTA);
                gfx->printf("%+6.2f", getHWT906RollQFiltered());
                
                // Quaternioni raw
                gfx->fillRect(10, 245, 220, 12, RGB565_BLUE);
                gfx->setCursor(10, 245);
                gfx->setTextColor(YELLOW);
                gfx->printf("%.2f %.2f %.2f %.2f", 
                           getHWT906QuaternionW(), getHWT906QuaternionX(), 
                           getHWT906QuaternionY(), getHWT906QuaternionZ());
                
                // Update status indicator
                bool gimbalLock = isHWT906GimbalLockDetected();
                gfx->fillRect(200, 20, 40, 20, gimbalLock ? RED : GREEN);
                gfx->setCursor(205, 25);
                gfx->setTextSize(1);
                gfx->setTextColor(WHITE);
                gfx->println(gimbalLock ? "GL" : "DUAL");
                
                lastUpdate = millis();
            }
            
            // Check touch
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= BACK_X && p.x <= (BACK_X + BACK_W) && 
                    p.y >= BACK_Y && p.y <= (BACK_Y + BACK_H)) {
                    delay(200);
                    actionRunning = false;
                    setCurrentMenuState(SUBMENU_1);
                    drawMenu(gfx, ui, SUBMENU_1);
                    return;
                }
            }
            
            delay(50);
        }
    }
    
    void showLiveGraph() {
        Serial.println("Showing live graph...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(30, 20);
        gfx->println("LIVE GRAPH");
        
        // Disegna assi
        gfx->drawLine(30, 240, 230, 240, WHITE);
        gfx->drawLine(30, 60, 30, 240, WHITE);
        
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
            float distance = 500.0 + sin(millis() / 1000.0) * 200;
            float y = 240 - (distance / 1000.0 * 180);
            
            if (x > 31) {
                gfx->drawLine(x-1, lastY, x, y, GREEN);
            }
            
            lastY = y;
            x++;
            
            if (x > 230) {
                gfx->fillRect(31, 61, 199, 179, BLACK);
                x = 31;
            }
            
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 150 && p.x <= 230 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    stopRequested = true;
                    setCurrentMenuState(SUBMENU_1);
                }
            }
            
            delay(50);
        }
        
        stopRequested = false;
        drawMenu(gfx, ui, getCurrentMenuState());
    }
    
    void showHWT906Config() {
        gfx->fillScreen(RGB565_BLACK);
        
        gfx->setCursor(20, 20);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("HWT906 CONFIG");
        
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
        
        gfx->setCursor(20, 190);
        gfx->setTextColor(GREEN);
        gfx->println("Dual-Mode: ACTIVE (Euler + Quaternions)");
        
        gfx->setCursor(20, 210);
        gfx->setTextColor(WHITE);
        gfx->println("Drift Compensation: ENABLED");
        
        // BACK button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    showHWT906Status();
                    return;
                }
            }
            delay(50);
        }
    }
    
    void showHWT906Calib() {
        gfx->fillScreen(RGB565_BLACK);
        
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
            
            // Enhanced calibration status
            gfx->setCursor(20, 140);
            gfx->setTextColor(YELLOW);
            gfx->println("SENSOR CALIBRATIONS:");
            
            // Gyro status
            gfx->setCursor(20, 160);
            gfx->setTextColor(hwt906.isGyroCalibrated() ? GREEN : RED);
            gfx->printf("Gyro: %s", hwt906.isGyroCalibrated() ? "CALIBRATED" : "NOT CALIBRATED");
            if (hwt906.isGyroCalibrated()) {
                float bias_x, bias_y, bias_z;
                uint32_t timestamp;
                hwt906.getGyroCalibrationInfo(bias_x, bias_y, bias_z, timestamp);
                gfx->setCursor(20, 175);
                gfx->setTextSize(1);
                gfx->printf("  Bias: %.3f %.3f %.3f °/s", bias_x, bias_y, bias_z);
            }
            
            // Accel status
            gfx->setCursor(20, 190);
            gfx->setTextColor(hwt906.isAccelCalibrated() ? GREEN : RED);
            gfx->printf("Accel: %s", hwt906.isAccelCalibrated() ? "CALIBRATED" : "NOT CALIBRATED");
            if (hwt906.isAccelCalibrated()) {
                float offset_x, offset_y, offset_z, scale_x, scale_y, scale_z;
                uint32_t timestamp;
                hwt906.getAccelCalibrationInfo(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, timestamp);
                gfx->setCursor(20, 205);
                gfx->setTextSize(1);
                gfx->printf("  Offset: %.3f %.3f %.3f g", offset_x, offset_y, offset_z);
            }
            
            // Mag status
            gfx->setCursor(20, 220);
            gfx->setTextColor(hwt906.isMagCalibrated() ? GREEN : RED);
            gfx->printf("Mag: %s", hwt906.isMagCalibrated() ? "CALIBRATED" : "NOT CALIBRATED");
            if (hwt906.isMagCalibrated()) {
                float hard_x, hard_y, hard_z;
                uint32_t timestamp;
                hwt906.getMagCalibrationInfo(hard_x, hard_y, hard_z, timestamp);
                gfx->setCursor(20, 235);
                gfx->setTextSize(1);
                gfx->printf("  Hard Iron: %.1f %.1f %.1f", hard_x, hard_y, hard_z);
            }
            
            // Gimbal lock and quaternion status
            gfx->setCursor(20, 250);
            gfx->setTextColor(data.gimbal_lock_detected ? RED : GREEN);
            gfx->printf("Gimbal Lock: %s", data.gimbal_lock_detected ? "DETECTED" : "OK");
            
            // Check touch
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    showHWT906Status();
                    return;
                }
            }
            delay(200);
        }
    }
    
    // === STEP CAL: FIXED CALIBRATION IMPLEMENTATIONS ===
    
    void calibrateGyro() {
        Serial.println("🎯 Starting Gyro Calibration...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(30, 80);
        gfx->println("GYRO CALIBRATION");
        
        gfx->setTextSize(1);
        gfx->setCursor(20, 120);
        gfx->setTextColor(YELLOW);
        gfx->println("Keep sensor PERFECTLY STILL");
        gfx->setCursor(20, 140);
        // 🔧 CRITICAL FIX: Tempo raddoppiato da 10 a 50 secondi
        gfx->println("for 50 seconds...");
        
        // Cancel button
        gfx->fillRect(70, 250, 100, 40, RED);
        gfx->drawRect(70, 250, 100, 40, WHITE);
        gfx->setCursor(95, 265);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("CANCEL");
        
        // Start calibration using wrapper function
        if (!hwt906.startGyroCalibration()) {
            gfx->fillScreen(BLACK);
            gfx->setCursor(50, 150);
            gfx->setTextColor(RED);
            gfx->setTextSize(2);
            gfx->println("CALIBRATION FAILED");
            gfx->setCursor(30, 180);
            gfx->setTextSize(1);
            gfx->println("Check sensor connection");
            delay(3000);
            setCurrentMenuState(SUBMENU_2);
            drawMenu(gfx, ui, SUBMENU_2);
            return;
        }
        
        Serial.println("✅ Gyro calibration started successfully");
        
        // Progress loop - FIX: use correct wrapper functions
        while (hwt906.isGyroCalibrationInProgress()) {
            // FIX: Call update to process calibration
            updateHWT906();
            
            float progress = hwt906.getGyroCalibrationProgress();
            
            // Progress bar
            int barWidth = (int)(200 * progress / 100.0f);
            gfx->fillRect(20, 180, 200, 20, BLACK);  // Clear previous
            gfx->fillRect(20, 180, barWidth, 20, GREEN);
            gfx->drawRect(20, 180, 200, 20, WHITE);
            
            // Percentage
            gfx->fillRect(90, 155, 60, 20, BLACK);
            gfx->setCursor(95, 158);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("%.0f%%", progress);
            
            // Debug info
            Serial.printf("📄 Gyro calibration progress: %.1f%%\n", progress);
            
            // Check for cancel
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 70 && p.x <= 170 && p.y >= 250 && p.y <= 290) {
                    hwt906.cancelGyroCalibration();
                    Serial.println("🚫 Gyro calibration cancelled by user");
                    break;
                }
            }
            
            delay(100);
        }
        
        // Show result
        gfx->fillRect(20, 220, 200, 60, BLACK);
        if (hwt906.isGyroCalibrated()) {
            gfx->setCursor(70, 230);
            gfx->setTextColor(GREEN);
            gfx->setTextSize(2);
            gfx->println("COMPLETE!");
            
            float bias_x, bias_y, bias_z;
            uint32_t timestamp;
            hwt906.getGyroCalibrationInfo(bias_x, bias_y, bias_z, timestamp);
            gfx->setCursor(20, 250);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("Bias: %.3f %.3f %.3f °/s", bias_x, bias_y, bias_z);
            
            Serial.printf("✅ Gyro calibration completed: bias=(%.3f, %.3f, %.3f) °/s\n", bias_x, bias_y, bias_z);
        } else {
            gfx->setCursor(70, 240);
            gfx->setTextColor(RED);
            gfx->setTextSize(2);
            gfx->println("CANCELLED");
            Serial.println("❌ Gyro calibration was cancelled");
        }
        
        delay(3000);
        setCurrentMenuState(SUBMENU_2);
        drawMenu(gfx, ui, SUBMENU_2);
    }
    
    void calibrateAccel() {
        Serial.println("🎯 Starting Accelerometer Calibration...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(20, 40);
        gfx->println("ACCEL CALIBRATION");
        
        gfx->setTextSize(1);
        gfx->setCursor(20, 80);
        gfx->setTextColor(YELLOW);
        gfx->println("6-Point Calibration Required");
        gfx->setCursor(20, 100);
        gfx->println("Follow instructions for each");
        gfx->setCursor(20, 115);
        gfx->println("orientation...");
        
        // Start button
        gfx->fillRect(50, 150, 80, 40, GREEN);
        gfx->drawRect(50, 150, 80, 40, WHITE);
        gfx->setCursor(75, 165);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("START");
        
        // Cancel button
        gfx->fillRect(150, 150, 80, 40, RED);
        gfx->drawRect(150, 150, 80, 40, WHITE);
        gfx->setCursor(170, 165);
        gfx->println("CANCEL");
        
        // Wait for user choice
        bool started = false;
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                if (p.x >= 50 && p.x <= 130 && p.y >= 150 && p.y <= 190) {
                    started = true;
                    delay(300);  // Longer delay to avoid double-tap
                    break;
                }
                
                if (p.x >= 150 && p.x <= 230 && p.y >= 150 && p.y <= 190) {
                    setCurrentMenuState(SUBMENU_2);
                    drawMenu(gfx, ui, SUBMENU_2);
                    return;
                }
            }
            delay(50);
        }
        
        if (!started) return;
        
        // Start calibration using wrapper function
        if (!hwt906.startAccelCalibration()) {
            gfx->fillScreen(BLACK);
            gfx->setCursor(50, 150);
            gfx->setTextColor(RED);
            gfx->setTextSize(2);
            gfx->println("CALIBRATION FAILED");
            gfx->setCursor(30, 180);
            gfx->setTextSize(1);
            gfx->println("Check sensor connection");
            delay(3000);
            setCurrentMenuState(SUBMENU_2);
            drawMenu(gfx, ui, SUBMENU_2);
            return;
        }
        
        Serial.println("✅ Accel calibration started successfully");
        
        // 6-point calibration loop - FIX: use correct wrapper functions
        while (hwt906.isAccelCalibrationInProgress()) {
            // FIX: Call update to process calibration
            updateHWT906();
            
            gfx->fillScreen(BLACK);
            
            gfx->setCursor(20, 20);
            gfx->setTextSize(2);
            gfx->setTextColor(WHITE);
            gfx->println("ACCEL CALIBRATION");
            
            // Progress
            float progress = hwt906.getAccelCalibrationProgress();
            int current_point = hwt906.getCurrentAccelPoint();
            
            gfx->setCursor(20, 60);
            gfx->setTextSize(1);
            gfx->setTextColor(YELLOW);
            gfx->printf("Point %d/6 (%.0f%%)", current_point, progress);
            
            // Current orientation instruction
            gfx->setCursor(20, 90);
            gfx->setTextColor(WHITE);
            gfx->println("Place sensor:");
            gfx->setCursor(20, 110);
            gfx->setTextColor(CYAN);
            gfx->println(hwt906.getCurrentAccelPointName());
            
            // Stability indicator
            bool is_ready = hwt906.isAccelPointReady();
            gfx->setCursor(20, 140);
            gfx->setTextColor(is_ready ? GREEN : RED);
            gfx->printf("Stability: %s", is_ready ? "READY" : "STABILIZING...");
            
            // FIX: Confirm button with better touch area and visual feedback
            if (is_ready) {
                gfx->fillRect(50, 170, 140, 50, GREEN);  // Larger button
                gfx->drawRect(50, 170, 140, 50, WHITE);
                gfx->fillRect(52, 172, 136, 46, GREEN);  // Double border for emphasis
                gfx->setCursor(85, 185);
                gfx->setTextSize(2);
                gfx->setTextColor(WHITE);
                gfx->println("CONFIRM");
                gfx->setCursor(75, 205);
                gfx->setTextSize(1);
                gfx->println("Touch to confirm");
            } else {
                gfx->fillRect(50, 170, 140, 50, DARKGREY);
                gfx->drawRect(50, 170, 140, 50, WHITE);
                gfx->setCursor(85, 185);
                gfx->setTextSize(2);
                gfx->setTextColor(LIGHTGREY);
                gfx->println("WAITING");
                gfx->setCursor(80, 205);
                gfx->setTextSize(1);
                gfx->println("Stabilizing...");
            }
            
            // Cancel button
            gfx->fillRect(70, 250, 100, 40, RED);
            gfx->drawRect(70, 250, 100, 40, WHITE);
            gfx->setCursor(95, 265);
            gfx->setTextSize(2);
            gfx->setTextColor(WHITE);
            gfx->println("CANCEL");
            
            // Debug info
            Serial.printf("📄 Accel point %d/6 - Ready: %s\n", current_point, is_ready ? "YES" : "NO");
            
            // FIX: Better touch handling with debouncing
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                // Confirm point - FIX: larger touch area and only when ready
                if (is_ready && p.x >= 50 && p.x <= 190 && p.y >= 170 && p.y <= 220) {
                    Serial.printf("✅ Confirming accel point %d\n", current_point);
                    
                    // Visual feedback
                    gfx->fillRect(50, 170, 140, 50, YELLOW);
                    gfx->setCursor(85, 190);
                    gfx->setTextColor(BLACK);
                    gfx->println("CONFIRMED!");
                    delay(500);
                    
                    hwt906.confirmAccelPoint();
                    delay(1000);  // Wait for confirmation processing
                }
                
                // Cancel
                if (p.x >= 70 && p.x <= 170 && p.y >= 250 && p.y <= 290) {
                    hwt906.cancelAccelCalibration();
                    Serial.println("🚫 Accelerometer calibration cancelled by user");
                    break;
                }
            }
            
            delay(100);
        }
        
        // Show result
        gfx->fillScreen(BLACK);
        gfx->setCursor(20, 100);
        gfx->setTextSize(2);
        
        if (hwt906.isAccelCalibrated()) {
            gfx->setTextColor(GREEN);
            gfx->println("ACCEL COMPLETE!");
            
            float offset_x, offset_y, offset_z, scale_x, scale_y, scale_z;
            uint32_t timestamp;
            hwt906.getAccelCalibrationInfo(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, timestamp);
            
            gfx->setCursor(20, 140);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("Offset: %.3f %.3f %.3f g", offset_x, offset_y, offset_z);
            gfx->setCursor(20, 160);
            gfx->printf("Scale:  %.3f %.3f %.3f", scale_x, scale_y, scale_z);
            
            Serial.printf("✅ Accel calibration completed\n");
        } else {
            gfx->setTextColor(RED);
            gfx->println("CANCELLED");
            Serial.println("❌ Accel calibration was cancelled");
        }
        
        delay(3000);
        setCurrentMenuState(SUBMENU_2);
        drawMenu(gfx, ui, SUBMENU_2);
    }
    
    void calibrateMag() {
        Serial.println("🎯 Starting Magnetometer Calibration...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(20, 60);
        gfx->println("MAG CALIBRATION");
        
        gfx->setTextSize(1);
        gfx->setCursor(20, 100);
        gfx->setTextColor(YELLOW);
        gfx->println("Rotate sensor slowly in ALL");
        gfx->setCursor(20, 120);
        // 🔧 CRITICAL FIX: Tempo raddoppiato da 30 a 100 secondi
        gfx->println("directions for 100 seconds");
        gfx->setCursor(20, 140);
        gfx->println("Include figure-8 motions");
        
        // Start button
        gfx->fillRect(50, 180, 80, 40, GREEN);
        gfx->drawRect(50, 180, 80, 40, WHITE);
        gfx->setCursor(75, 195);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("START");
        
        // Cancel button
        gfx->fillRect(150, 180, 80, 40, RED);
        gfx->drawRect(150, 180, 80, 40, WHITE);
        gfx->setCursor(170, 195);
        gfx->println("CANCEL");
        
        // Wait for user choice
        bool started = false;
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                if (p.x >= 50 && p.x <= 130 && p.y >= 180 && p.y <= 220) {
                    started = true;
                    delay(300);  // Longer delay to avoid double-tap
                    break;
                }
                
                if (p.x >= 150 && p.x <= 230 && p.y >= 180 && p.y <= 220) {
                    setCurrentMenuState(SUBMENU_2);
                    drawMenu(gfx, ui, SUBMENU_2);
                    return;
                }
            }
            delay(50);
        }
        
        if (!started) return;
        
        // Start calibration using wrapper function
        if (!hwt906.startMagCalibration()) {
            gfx->fillScreen(BLACK);
            gfx->setCursor(50, 150);
            gfx->setTextColor(RED);
            gfx->setTextSize(2);
            gfx->println("CALIBRATION FAILED");
            gfx->setCursor(30, 180);
            gfx->setTextSize(1);
            gfx->println("Check sensor connection");
            delay(3000);
            setCurrentMenuState(SUBMENU_2);
            drawMenu(gfx, ui, SUBMENU_2);
            return;
        }
        
        Serial.println("✅ Mag calibration started successfully");
        
        // FIX: Get start time for countdown calculation
        uint32_t cal_start_time = millis();
        
        // Progress loop - FIX: use correct wrapper functions
        while (hwt906.isMagCalibrationInProgress()) {
            // FIX: Call update to process calibration
            updateHWT906();
            
            gfx->fillScreen(BLACK);
            
            gfx->setCursor(20, 40);
            gfx->setTextSize(2);
            gfx->setTextColor(WHITE);
            gfx->println("MAG CALIBRATION");
            
            gfx->setCursor(20, 80);
            gfx->setTextSize(1);
            gfx->setTextColor(YELLOW);
            gfx->println("Keep rotating in all directions...");
            
            float progress = hwt906.getMagCalibrationProgress();
            
            // Progress bar
            int barWidth = (int)(200 * progress / 100.0f);
            gfx->fillRect(20, 120, 200, 20, BLACK);  // Clear previous
            gfx->fillRect(20, 120, barWidth, 20, GREEN);
            gfx->drawRect(20, 120, 200, 20, WHITE);
            
            // Percentage
            gfx->fillRect(90, 95, 60, 20, BLACK);
            gfx->setCursor(95, 98);
            gfx->setTextColor(WHITE);
            gfx->printf("%.0f%%", progress);
            
            // FIX: Calculate actual time remaining based on elapsed time
            uint32_t elapsed_time = (millis() - cal_start_time) / 1000;  // seconds
            // 🔧 CRITICAL FIX: Aggiornato timeout da 30 a 100 secondi
            int time_remaining = max(0, 100 - (int)elapsed_time);  // 100 second calibration
            gfx->fillRect(20, 150, 200, 15, BLACK);  // Clear previous
            gfx->setCursor(20, 150);
            gfx->printf("Time remaining: %ds", time_remaining);
            
            // Cancel button
            gfx->fillRect(70, 250, 100, 40, RED);
            gfx->drawRect(70, 250, 100, 40, WHITE);
            gfx->setCursor(95, 265);
            gfx->setTextSize(2);
            gfx->setTextColor(WHITE);
            gfx->println("CANCEL");
            
            // Debug info
            Serial.printf("📄 Mag calibration progress: %.1f%% - Time remaining: %ds\n", progress, time_remaining);
            
            // Check for cancel
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 70 && p.x <= 170 && p.y >= 250 && p.y <= 290) {
                    hwt906.cancelMagCalibration();
                    Serial.println("🚫 Magnetometer calibration cancelled by user");
                    break;
                }
            }
            
            delay(100);
        }
        
        // Show result
        gfx->fillScreen(BLACK);
        gfx->setCursor(20, 100);
        gfx->setTextSize(2);
        
        if (hwt906.isMagCalibrated()) {
            gfx->setTextColor(GREEN);
            gfx->println("MAG COMPLETE!");
            
            float hard_x, hard_y, hard_z;
            uint32_t timestamp;
            hwt906.getMagCalibrationInfo(hard_x, hard_y, hard_z, timestamp);
            
            gfx->setCursor(20, 140);
            gfx->setTextSize(1);
            gfx->setTextColor(WHITE);
            gfx->printf("Hard Iron: %.1f %.1f %.1f", hard_x, hard_y, hard_z);
            
            Serial.printf("✅ Mag calibration completed\n");
        } else {
            gfx->setTextColor(RED);
            gfx->println("CANCELLED");
            Serial.println("❌ Mag calibration was cancelled");
        }
        
        delay(3000);
        setCurrentMenuState(SUBMENU_2);
        drawMenu(gfx, ui, SUBMENU_2);
    }
    
    // === SUBMENU 3: SERVICE (unchanged functionality) ===
    
    void changeSettings() {
        Serial.println("Opening settings...");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(50, 20);
        gfx->println("SETTINGS");
        
        gfx->setTextSize(1);
        gfx->setCursor(20, 80);
        gfx->println("1. Dead Zones");
        gfx->setCursor(20, 100);
        gfx->println("2. Filter Parameters");
        gfx->setCursor(20, 120);
        gfx->println("3. OM60 Range");
        gfx->setCursor(20, 140);
        gfx->println("4. Sample Rate");
        
        // Enhanced settings
        gfx->setCursor(20, 170);
        gfx->setTextColor(YELLOW);
        gfx->println("5. Reset All Calibrations");
        gfx->setCursor(20, 190);
        gfx->println("6. Reset Individual Calibrations");
        
        // Back button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                if (p.x >= 80 && p.x <= 160 && p.y >= 270 && p.y <= 305) {
                    delay(200);
                    break;
                }
                
                // Option 5: Reset all calibrations
                if (p.x >= 20 && p.x <= 220 && p.y >= 170 && p.y <= 185) {
                    delay(200);
                    
                    // Confirmation dialog
                    gfx->fillRect(40, 120, 160, 80, RED);
                    gfx->drawRect(40, 120, 160, 80, WHITE);
                    gfx->setCursor(50, 135);
                    gfx->setTextSize(1);
                    gfx->setTextColor(WHITE);
                    gfx->println("Reset ALL calibrations?");
                    gfx->setCursor(70, 155);
                    gfx->println("YES        NO");
                    
                    while (true) {
                        if (touch->getTouches() > 0) {
                            auto p2 = touch->touchPoints[0];
                            if (p2.x >= 70 && p2.x <= 100 && p2.y >= 155 && p2.y <= 170) {
                                // YES
                                hwt906.resetAllSensorCalibrations();
                                gfx->fillRect(50, 175, 140, 20, BLACK);
                                gfx->setCursor(60, 178);
                                gfx->setTextColor(GREEN);
                                gfx->println("ALL CALIBRATIONS RESET");
                                delay(2000);
                                break;
                            }
                            if (p2.x >= 130 && p2.x <= 160 && p2.y >= 155 && p2.y <= 170) {
                                // NO
                                break;
                            }
                        }
                        delay(50);
                    }
                    
                    // Redraw settings screen
                    gfx->fillRect(40, 120, 160, 80, BLACK);
                    gfx->setCursor(20, 170);
                    gfx->setTextColor(YELLOW);
                    gfx->println("5. Reset All Calibrations");
                }
            }
            delay(50);
        }
        
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
        gfx->println("HySeq OM60/HWT906 v2.0 + StepCAL");
        
        gfx->setCursor(20, 80);
        gfx->printf("Free Heap: %lu KB", ESP.getFreeHeap() / 1024);
        
        gfx->setCursor(20, 100);
        gfx->printf("CPU Freq: %lu MHz", ESP.getCpuFreqMHz());
        
        gfx->setCursor(20, 120);
        gfx->printf("Flash: %lu MB", ESP.getFlashChipSize() / 1048576);
        
        // Sensor status
        gfx->setCursor(20, 150);
        gfx->print("HWT906: ");
        gfx->setTextColor(isHWT906Ready() ? GREEN : RED);
        gfx->println(isHWT906Ready() ? "OK" : "ERROR");
        
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 170);
        gfx->print("OM60: ");
        gfx->setTextColor(isOM60Ready() ? GREEN : RED);
        gfx->println(isOM60Ready() ? "OK" : "ERROR");
        
        // Enhanced system info
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 190);
        gfx->print("Dual-Mode: ");
        gfx->setTextColor(hwt906.getData().quaternion_mode_active ? GREEN : RED);
        gfx->println(hwt906.getData().quaternion_mode_active ? "ACTIVE" : "INACTIVE");
        
        // Calibration status summary
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, 210);
        gfx->printf("Calibrations: G:%s A:%s M:%s",
                   hwt906.isGyroCalibrated() ? "OK" : "NO",
                   hwt906.isAccelCalibrated() ? "OK" : "NO",
                   hwt906.isMagCalibrated() ? "OK" : "NO");
        
        // Live sensor data
        if (isHWT906Ready()) {
            gfx->setCursor(20, 230);
            gfx->printf("P: %.1f  Y: %.1f  GL: %s", getHWT906Pitch(), getHWT906Yaw(),
                       isHWT906GimbalLockDetected() ? "Y" : "N");
        }
        
        if (isOM60Ready()) {
            gfx->setCursor(20, 245);
            gfx->printf("Distance: %.0f mm", getFilteredOM60Distance());
        }
        
        // Back button
        gfx->fillRect(80, 270, 80, 35, ORANGE);
        gfx->drawRect(80, 270, 80, 35, WHITE);
        gfx->setCursor(105, 282);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("BACK");
        
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
        
        drawMenu(gfx, ui, SUBMENU_3);
    }
    
    void factoryReset() {
        Serial.println("Factory Reset!");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        gfx->setCursor(40, 80);
        gfx->println("CONFIRM RESET?");
        
        gfx->setTextColor(WHITE);
        gfx->setTextSize(1);
        gfx->setCursor(30, 120);
        gfx->println("This will erase ALL data!");
        gfx->setCursor(30, 140);
        gfx->setTextColor(RED);
        gfx->println("Including calibrations!");
        
        // Buttons
        gfx->fillRect(40, 180, 60, 40, RED);
        gfx->drawRect(40, 180, 60, 40, WHITE);
        gfx->setCursor(55, 195);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->println("YES");
        
        gfx->fillRect(140, 180, 60, 40, GREEN);
        gfx->drawRect(140, 180, 60, 40, WHITE);
        gfx->setCursor(160, 195);
        gfx->println("NO");
        
        bool confirmed = false;
        
        while (true) {
            if (touch->getTouches() > 0) {
                auto p = touch->touchPoints[0];
                
                if (p.x >= 40 && p.x <= 100 && p.y >= 180 && p.y <= 220) {
                    confirmed = true;
                    delay(200);
                    break;
                }
                
                if (p.x >= 140 && p.x <= 200 && p.y >= 180 && p.y <= 220) {
                    confirmed = false;
                    delay(200);
                    break;
                }
            }
            delay(50);
        }
        
        if (confirmed) {
            gfx->fillScreen(BLACK);
            gfx->setTextColor(RED);
            gfx->setTextSize(2);
            gfx->setCursor(50, 120);
            gfx->println("RESETTING...");
            
            // Complete factory reset
            hwt906.resetAllSensorCalibrations();
            hwt906.resetCalibration();
            
            gfx->setCursor(40, 160);
            gfx->println("COMPLETE!");
            gfx->setCursor(30, 200);
            gfx->println("Rebooting...");
            
            delay(2000);
            ESP.restart();
        } else {
            gfx->fillScreen(BLACK);
            gfx->setTextColor(GREEN);
            gfx->setCursor(60, 150);
            gfx->setTextSize(2);
            gfx->println("CANCELLED");
            delay(1000);
            
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