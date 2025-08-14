// diagnostic_menu.cpp
// Implementazioni del menu diagnostico HWT906

#include "diagnostic_menu.h"

// === IMPLEMENTAZIONE CLASSE ===

DiagnosticMenu::DiagnosticMenu(Arduino_GFX* display, CSE_CST328* touchpad) 
    : gfx(display), touch(touchpad), currentState(DIAG_EXIT), 
      lastUpdate(0), selectedOption(0), needsRedraw(true) {
    memset(&tempConfig, 0, sizeof(tempConfig));
    memset(&liveStats, 0, sizeof(liveStats));
}

void DiagnosticMenu::show() {
	// AGGIUNGI QUESTA RIGA:
    gfx->fillScreen(DIAG_BG_COLOR);  // Pulisce tutto lo schermo
    currentState = DIAG_MAIN;
    selectedOption = 0;
    needsRedraw = true;
    loadCurrentConfig();
    updateLiveStats();
    
    Serial.println("🔬 Diagnostic Menu activated");
}

void DiagnosticMenu::update() {
    if (currentState == DIAG_EXIT) return;
    
    uint32_t now = millis();
    
    // Update dati live ogni 100ms
    if (now - lastUpdate >= DIAG_UPDATE_INTERVAL) {
        updateLiveStats();
        lastUpdate = now;
        
        // Ridisegna solo valori live sulla schermata principale
        if (currentState == DIAG_MAIN) {
            drawMainScreen();
        }
    }
    
    // Gestisci touch input
    handleTouch();
    
    // Ridisegna se necessario
    if (needsRedraw) {
        switch (currentState) {
            case DIAG_MAIN:   drawMainScreen();   break;
            case DIAG_CONFIG: drawConfigScreen(); break;
            case DIAG_CALIB:  drawCalibScreen();  break;
            case DIAG_RESET:  drawResetScreen();  break;
            default: break;
        }
        needsRedraw = false;
    }
}

void DiagnosticMenu::updateLiveStats() {
    // Ottieni dati HWT906
    liveStats.current_pitch = getHWT906Pitch();
    liveStats.current_yaw = getHWT906Yaw();
    liveStats.current_roll = getHWT906Roll();
    liveStats.connected = isHWT906Ready();
    liveStats.update_rate = getHWT906UpdateRate();
    
    // Ottieni dati completi dalla classe
    const HWT906Data& data = hwt906.getData();
    liveStats.frame_count = data.frame_count;
    liveStats.error_frames = data.error_frames;
    liveStats.error_percentage = data.frame_count > 0 ? 
        100.0f * data.error_frames / data.frame_count : 0.0f;
    
    // Range min/max
    liveStats.pitch_min = data.pitch_min;
    liveStats.pitch_max = data.pitch_max;
    liveStats.yaw_min = data.yaw_min;
    liveStats.yaw_max = data.yaw_max;
    
    // Tempo statico
    liveStats.static_time = data.is_static ? 
        (millis() - data.static_start_time) / 1000 : 0;
    
    // Stabilità (placeholder - implementare se disponibile)
    liveStats.pitch_sigma = 0.1f;  // Da implementare
    liveStats.yaw_sigma = 0.1f;    // Da implementare
}

void DiagnosticMenu::drawMainScreen() {
    // Header
    drawHeader("-*- HWT906 DIAGNOSTICS");
    
    int y = DIAG_HEADER_HEIGHT + 10;
    
    // Status connection
    gfx->setCursor(10, y);
    gfx->setTextSize(1);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("📊 STATUS:");
    gfx->setCursor(80, y);
    gfx->setTextColor(liveStats.connected ? DIAG_STATUS_OK : DIAG_STATUS_ERROR);
    gfx->print(liveStats.connected ? "✅ CONNECTED" : "❌ OFFLINE");
    
    y += 20;
    
    // Modalità e Baud
    drawValueLine(y, "🔗 Mode:", getPrecisionModeString(tempConfig.precision_mode));
    y += 15;
    drawValueLine(y, "📡 Baud:", getBaudRateString(tempConfig.baud_rate));
    y += 20;
    
    // Valori live P/Y
    char value_str[50];  // Buffer aumentato
    snprintf(value_str, sizeof(value_str), "P:%+6.1f Y:%+06.1f", 
             liveStats.current_pitch, liveStats.current_yaw);
    drawValueLine(y, "🎯 PY:", value_str);
    y += 20;
    
    // Range corrente
    gfx->setCursor(10, y);
    gfx->setTextSize(1);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("📈 RANGE CORRENTE:");
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%+6.1f ÷ %+6.1f ", 
             liveStats.pitch_min, liveStats.pitch_max);
    drawValueLine(y, "  Pitch:", value_str);
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%+6.1f ÷ %+6.1f", 
             liveStats.yaw_min, liveStats.yaw_max);
    drawValueLine(y, "  Yaw:", value_str);
    y += 20;
    
    // Performance stats
    snprintf(value_str, sizeof(value_str), "%.1fHz (%.1f%% OK)", 
             liveStats.update_rate, 100.0f - liveStats.error_percentage);
    drawValueLine(y, "🔄 UPDATE:", value_str);
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%lu (Err: %lu)", 
             liveStats.frame_count, liveStats.error_frames);
    drawValueLine(y, "📦 Frames:", value_str);
    
    // Footer buttons
    const char* buttons[] = {"CONFIG", "CALIB", "RESET", "BACK"};
    drawFooter(buttons, 4);
}

void DiagnosticMenu::drawConfigScreen() {
    drawHeader(" HWT906 CONFIG");
    
    int y = DIAG_HEADER_HEIGHT + 10;
    
    // Precision Mode Selection
    gfx->setCursor(10, y);
    gfx->setTextSize(1);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("🎚️ PRECISION MODE:");
    y += 20;
    
    const char* modes[] = {"Static Low (5Hz)", "Static Med (10Hz)", 
                          "Static High (20Hz)", "Dynamic (50Hz)"};
    
    for (int i = 0; i < 4; i++) {
        gfx->setCursor(15, y);
        gfx->setTextColor(tempConfig.precision_mode == i ? DIAG_BUTTON_ACTIVE : DIAG_TEXT_COLOR);
        gfx->printf("%s %s", (tempConfig.precision_mode == i) ? "*" : " ", modes[i]);  // Caratteri semplici
        y += 15;
    }
    
    y += 10;
    
    // Baud Rate Selection  
    gfx->setCursor(10, y);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("📡 BAUD RATE:");
    y += 20;
    
    uint32_t bauds[] = {9600, 115200, 921600};
    const char* baud_labels[] = {"9600 (Standard)", "115200 (Fast)", "921600 (Ultra)"};
    
    for (int i = 0; i < 3; i++) {
        gfx->setCursor(15, y);
        gfx->setTextColor(tempConfig.baud_rate == bauds[i] ? DIAG_BUTTON_ACTIVE : DIAG_TEXT_COLOR);
        gfx->printf("%s %s", (tempConfig.baud_rate == bauds[i]) ? "*" : " ", baud_labels[i]);  // Caratteri semplici
        y += 15;
    }
    
    y += 10;
    
    // Dead Zones
    char value_str[50];
    snprintf(value_str, sizeof(value_str), "P:%.2f Y:%.2f", 
             tempConfig.pitch_dead_zone, tempConfig.yaw_dead_zone);
    drawValueLine(y, "🎯 DEAD ZONES:", value_str);
    y += 20;
    
    // Drift Compensation
    drawValueLine(y, "🔄 DRIFT COMP:", tempConfig.drift_compensation ? "ON" : "OFF");
    
    // Status modifiche
    if (tempConfig.modified) {
        gfx->setCursor(10, 260);
        gfx->setTextColor(DIAG_STATUS_WARNING);
        gfx->setTextSize(1);
        gfx->print("⚠️ Modifiche non salvate!");
    }
    
    const char* buttons[] = {"APPLY", "DEFAULTS", "BACK"};
    drawFooter(buttons, 3);
}

void DiagnosticMenu::drawCalibScreen() {
    drawHeader(" HWT906 CALIBRATION");
    
    int y = DIAG_HEADER_HEIGHT + 10;
    
    // Current offsets
    const HWT906Data& data = hwt906.getData();
    
    gfx->setCursor(10, y);
    gfx->setTextSize(1);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("📐 CURRENT OFFSET:");
    y += 20;
    
    char value_str[30];
    snprintf(value_str, sizeof(value_str), "%+6.2f", data.pitch_offset);
    drawValueLine(y, "  Pitch:", value_str);
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%+6.2f", data.yaw_offset);
    drawValueLine(y, "  Yaw:", value_str);
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%+6.2f", data.roll_offset);
    drawValueLine(y, "  Roll:", value_str);
    y += 20;
    
    // Raw values (live)
    gfx->setCursor(10, y);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("🎚️ RAW VALUES:");
    y += 20;
    
    snprintf(value_str, sizeof(value_str), "%+6.2f (Live)", data.pitch_raw);
    drawValueLine(y, "  Pitch:", value_str);
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%+6.2f (Live)", data.yaw_raw);
    drawValueLine(y, "  Yaw:", value_str);
    y += 15;
    
    snprintf(value_str, sizeof(value_str), "%+6.2f (Live)", data.roll_raw);
    drawValueLine(y, "  Roll:", value_str);
    y += 20;
    
    // Stability
    snprintf(value_str, sizeof(value_str), "P:%.3f Y:%.3f", 
             liveStats.pitch_sigma, liveStats.yaw_sigma);
    drawValueLine(y, "📊 STABILITY σ:", value_str);
    y += 20;
    
    // Static time
    snprintf(value_str, sizeof(value_str), "%02lu:%02lu sec", 
             liveStats.static_time / 60, liveStats.static_time % 60);
    drawValueLine(y, "⏱️ STATIC TIME:", value_str);
    
    const char* buttons[] = {"ZERO NOW", "RESET CAL", "BACK"};
    drawFooter(buttons, 3);
}

void DiagnosticMenu::drawResetScreen() {
    drawHeader(" HWT906 RESET");
    
    int y = DIAG_HEADER_HEIGHT + 15;
    
    gfx->setCursor(10, y);
    gfx->setTextSize(1);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print("🔄 RESET OPTIONS:");
    y += 25;
    
    // Reset options come bottoni
    drawButton(10, y, 100, 25, "RANGE STATS");
    drawButton(130, y, 100, 25, "CALIBRATION");
    y += 35;
    
    drawButton(10, y, 100, 25, "DRIFT COMP");
    drawButton(130, y, 100, 25, "STATISTICS");
    y += 35;
    
    drawButton(70, y, 100, 25, "FULL RESET", true);  // Evidenziato
    y += 35;
    
    // Warning
    gfx->setCursor(10, y);
    gfx->setTextColor(DIAG_STATUS_ERROR);
    gfx->print("⚠️ FULL RESET cancella tutto!");
    
    const char* buttons[] = {"CONFIRM", "CANCEL", "BACK"};
    drawFooter(buttons, 3);
}

// === UTILITY FUNCTIONS ===
void DiagnosticMenu::drawHeader(const char* title) {
    gfx->fillRect(0, 0, 240, DIAG_HEADER_HEIGHT, DIAG_HEADER_COLOR);
    gfx->setCursor(10, 8);
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->print(title);
}

void DiagnosticMenu::drawFooter(const char* buttons[], int count) {
    int y = 320 - DIAG_FOOTER_HEIGHT;
    gfx->fillRect(0, y, 240, DIAG_FOOTER_HEIGHT, DIAG_BG_COLOR);
    
    int button_width = 240 / count;
    for (int i = 0; i < count; i++) {
        drawButton(i * button_width + 5, y + 3, button_width - 10, 
                  DIAG_BUTTON_HEIGHT - 6, buttons[i]);
    }
}

void DiagnosticMenu::drawButton(int x, int y, int w, int h, const char* text, bool active) {
    uint16_t color = active ? DIAG_BUTTON_ACTIVE : DIAG_BUTTON_COLOR;
    
    gfx->fillRect(x, y, w, h, color);
    gfx->drawRect(x, y, w, h, WHITE);
    
    // Centra il testo
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    int text_x = x + (w - strlen(text) * 6) / 2;
    int text_y = y + (h - 8) / 2;
    gfx->setCursor(text_x, text_y);
    gfx->print(text);
}

void DiagnosticMenu::drawValueLine(int y, const char* label, const char* value, uint16_t color) {
    gfx->setCursor(10, y);
    gfx->setTextSize(1);
    gfx->setTextColor(DIAG_TEXT_COLOR);
    gfx->print(label);
    
    gfx->setCursor(90, y);
    gfx->setTextColor(color);
    gfx->print(value);
}

// === TOUCH HANDLING ===
void DiagnosticMenu::handleTouch() {
    if (touch->getTouches() == 0) return;
    
    auto p = touch->touchPoints[0];
    int x = p.x;
    int y = p.y;
    
    // Debounce
    static uint32_t lastTouch = 0;
    if (millis() - lastTouch < 200) return;
    lastTouch = millis();
    
    switch (currentState) {
        case DIAG_MAIN:   processMainTouch(x, y);   break;
        case DIAG_CONFIG: processConfigTouch(x, y); break;
        case DIAG_CALIB:  processCalibTouch(x, y);  break;
        case DIAG_RESET:  processResetTouch(x, y);  break;
        default: break;
    }
}

void DiagnosticMenu::processMainTouch(int x, int y) {
    // Footer buttons
    if (y >= 285) {
        if (x < 60) {          // CONFIG
            currentState = DIAG_CONFIG;
            needsRedraw = true;
        } else if (x < 120) {  // CALIB
            currentState = DIAG_CALIB;
            needsRedraw = true;
        } else if (x < 180) {  // RESET
            currentState = DIAG_RESET;
            needsRedraw = true;
        } else {               // BACK
            currentState = DIAG_EXIT;
        }
    }
}

void DiagnosticMenu::processConfigTouch(int x, int y) {
    // Precision mode selection (y: 45-105)
    if (y >= 45 && y <= 105 && x >= 15 && x <= 200) {
        int mode = (y - 45) / 15;
        if (mode >= 0 && mode <= 3) {
            tempConfig.precision_mode = (HWT906PrecisionMode)mode;
            tempConfig.modified = true;
            needsRedraw = true;
        }
    }
    
    // Baud rate selection (y: 125-170)
    if (y >= 125 && y <= 170 && x >= 15 && x <= 200) {
        int baud_idx = (y - 125) / 15;
        uint32_t bauds[] = {9600, 115200, 921600};
        if (baud_idx >= 0 && baud_idx <= 2) {
            tempConfig.baud_rate = bauds[baud_idx];
            tempConfig.modified = true;
            needsRedraw = true;
        }
    }
    
    // Footer buttons
    if (y >= 285) {
        if (x < 80) {          // APPLY
            applyTempConfig();
        } else if (x < 160) {  // DEFAULTS
            loadCurrentConfig();
            tempConfig.modified = false;
            needsRedraw = true;
        } else {               // BACK
            currentState = DIAG_MAIN;
            needsRedraw = true;
        }
    }
}

void DiagnosticMenu::processCalibTouch(int x, int y) {
    // Footer buttons
    if (y >= 285) {
        if (x < 80) {          // ZERO NOW
            zeroHWT906Angles();
            Serial.println("🎯 Zero calibration performed");
        } else if (x < 160) {  // RESET CAL
            resetHWT906Calibration();
            Serial.println("🔄 Calibration reset");
        } else {               // BACK
            currentState = DIAG_MAIN;
            needsRedraw = true;
        }
    }
}

void DiagnosticMenu::processResetTouch(int x, int y) {
    // Reset option buttons
    if (y >= 60 && y <= 85) {
        if (x >= 10 && x <= 110) {         // RANGE STATS
            performReset(1);
        } else if (x >= 130 && x <= 230) { // CALIBRATION
            performReset(2);
        }
    } else if (y >= 95 && y <= 120) {
        if (x >= 10 && x <= 110) {         // DRIFT COMP
            performReset(3);
        } else if (x >= 130 && x <= 230) { // STATISTICS
            performReset(4);
        }
    } else if (y >= 130 && y <= 155) {
        if (x >= 70 && x <= 170) {         // FULL RESET
            performReset(0);
        }
    }
    
    // Footer buttons
    if (y >= 285) {
        if (x < 80) {          // CONFIRM
            Serial.println("✅ Reset confirmed");
        } else if (x < 160) {  // CANCEL
            Serial.println("❌ Reset cancelled");
        }
        currentState = DIAG_MAIN;
        needsRedraw = true;
    }
}

// === AZIONI ===
void DiagnosticMenu::loadCurrentConfig() {
    // Carica configurazione attuale dal sistema
    tempConfig.precision_mode = HWT906_MODE_STATIC_MED;  // Default
    tempConfig.baud_rate = HWT906_UART_SPEED;  // ← LEGGE IL VALORE REALE DAL #define
    tempConfig.pitch_dead_zone = 0.1f;
    tempConfig.yaw_dead_zone = 0.15f;
    tempConfig.drift_compensation = true;
    tempConfig.modified = false;
}

void DiagnosticMenu::applyTempConfig() {
    // Applica configurazione temporanea al sistema
    configureHWT906Mode(tempConfig.precision_mode);
    enableHWT906DriftCompensation(tempConfig.drift_compensation);
    setHWT906DeadZones(tempConfig.pitch_dead_zone, tempConfig.yaw_dead_zone);
    
    tempConfig.modified = false;
    Serial.println("✅ Configuration applied");
    
    currentState = DIAG_MAIN;
    needsRedraw = true;
}

void DiagnosticMenu::performReset(uint8_t type) {
    switch (type) {
        case 0: // FULL RESET
            resetHWT906Range();
            resetHWT906Calibration();
            Serial.println("🔄 FULL RESET performed");
            break;
        case 1: // RANGE STATS
            resetHWT906Range();
            Serial.println("📊 Range stats reset");
            break;
        case 2: // CALIBRATION
            resetHWT906Calibration();
            Serial.println("🎯 Calibration reset");
            break;
        case 3: // DRIFT COMP
            // Reset drift compensation history
            Serial.println("🔄 Drift compensation reset");
            break;
        case 4: // STATISTICS
            // Reset frame statistics
            Serial.println("📊 Statistics reset");
            break;
    }
}

const char* DiagnosticMenu::getPrecisionModeString(HWT906PrecisionMode mode) {
    switch (mode) {
        case HWT906_MODE_STATIC_LOW:  return "Static Low";
        case HWT906_MODE_STATIC_MED:  return "Static Med";
        case HWT906_MODE_STATIC_HIGH: return "Static High";
        case HWT906_MODE_DYNAMIC:     return "Dynamic";
        default: return "Unknown";
    }
}

const char* DiagnosticMenu::getBaudRateString(uint32_t baud) {
    switch (baud) {
        case 9600:   return "9600";
        case 115200: return "115200";
        case 921600: return "921600";
        default: return "Custom";
    }
}
