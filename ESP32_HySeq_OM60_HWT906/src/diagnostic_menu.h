// diagnostic_menu.h
// Menu diagnostico completo per HWT906 - SOLO DICHIARAZIONI

#ifndef DIAGNOSTIC_MENU_H
#define DIAGNOSTIC_MENU_H

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <CSE_CST328.h>
#include "ui_config.h"
#include "hwt906_handler.h"
#include "hwt906_compat.h"

// === STATI MENU DIAGNOSTICO ===
enum DiagnosticState {
    DIAG_MAIN,          // Schermata principale live
    DIAG_CONFIG,        // Configurazione parametri
    DIAG_CALIB,         // Calibrazione e zero
    DIAG_RESET,         // Reset opzioni
    DIAG_EXIT           // Ritorna al menu principale
};

// === CONFIGURAZIONI UI ===
#define DIAG_UPDATE_INTERVAL 100   // 10Hz update live data
#define DIAG_HEADER_HEIGHT 25
#define DIAG_FOOTER_HEIGHT 35
#define DIAG_BUTTON_HEIGHT 30
#define DIAG_BUTTON_WIDTH 55

// === COLORI DIAGNOSTICO ===
#define DIAG_BG_COLOR       RGB565_BLACK
#define DIAG_HEADER_COLOR   RGB565_BLUE
#define DIAG_TEXT_COLOR     WHITE
#define DIAG_VALUE_COLOR    YELLOW
#define DIAG_STATUS_OK      GREEN
#define DIAG_STATUS_ERROR   RED
#define DIAG_STATUS_WARNING ORANGE
#define DIAG_BUTTON_COLOR   RGB565_DARKGREY
#define DIAG_BUTTON_ACTIVE  CYAN

// === CLASSE MENU DIAGNOSTICO ===
class DiagnosticMenu {
private:
    Arduino_GFX* gfx;
    CSE_CST328* touch;
    DiagnosticState currentState;
    
    // Stato UI
    uint32_t lastUpdate;
    uint8_t selectedOption;
    bool needsRedraw;
    
    // Configurazione temporanea (prima di Apply)
    struct TempConfig {
        HWT906PrecisionMode precision_mode;
        uint32_t baud_rate;
        float pitch_dead_zone;
        float yaw_dead_zone;
        bool drift_compensation;
        bool modified;
    } tempConfig;
    
    // Statistiche live
    struct LiveStats {
        float current_pitch;
        float current_yaw;
        float current_roll;
        float pitch_min, pitch_max;
        float yaw_min, yaw_max;
        float update_rate;
        uint32_t frame_count;
        uint32_t error_frames;
        float error_percentage;
        bool connected;
        uint32_t static_time;
        float pitch_sigma, yaw_sigma;
    } liveStats;

public:
    DiagnosticMenu(Arduino_GFX* display, CSE_CST328* touchpad);
    
    // Controllo menu
    void show();                    // Entra nel menu diagnostico
    void update();                  // Update loop (chiamare nel main loop)
    void handleTouch();             // Gestione touch
    bool isActive() { return currentState != DIAG_EXIT; }
    
private:
    // Rendering schermate - SOLO DICHIARAZIONI
    void drawMainScreen();
    void drawConfigScreen();
    void drawCalibScreen();
    void drawResetScreen();
    
    // Aggiornamento dati - SOLO DICHIARAZIONI
    void updateLiveStats();
    void loadCurrentConfig();
    void applyTempConfig();
    
    // Utilità rendering - SOLO DICHIARAZIONI
    void drawHeader(const char* title);
    void drawFooter(const char* buttons[], int count);
    void drawButton(int x, int y, int w, int h, const char* text, bool active = false);
    void drawStatusIndicator(int x, int y, const char* label, bool status);
    void drawValueLine(int y, const char* label, const char* value, uint16_t color = DIAG_VALUE_COLOR);
    
    // Gestione input - SOLO DICHIARAZIONI
    bool isButtonPressed(int x, int y, int w, int h, int touch_x, int touch_y);
    void processMainTouch(int x, int y);
    void processConfigTouch(int x, int y);
    void processCalibTouch(int x, int y);
    void processResetTouch(int x, int y);
    
    // Azioni - SOLO DICHIARAZIONI
    void performReset(uint8_t type);
    void saveConfiguration();
    const char* getPrecisionModeString(HWT906PrecisionMode mode);
    const char* getBaudRateString(uint32_t baud);
};

#endif // DIAGNOSTIC_MENU_H
