/*
 * ESP32 HySeq OM60-HWT906
 * Sistema di misura professionale con sensore laser OM60 e IMU HWT906
 * 
 * Hardware:
 * - ESP32-S3 con display touch 2.8"
 * - OM60: Sensore laser 200-1000mm (analogico 0-10V)
 * - HWT906: IMU 9-assi (I2C + UART)
 * 
 * Versione: 2.0
 * Data: Agosto 2025
 */


// Prima di tutti gli include
// #define USE_FREERTOS  // rimuovi il commento per attivare FreeRTOS

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <CSE_CST328.h>
#include <Wire.h>
//#include <EEPROM.h>  - rimosso e sostituito da Preferences.h in hwt906_handler.cpp

// === MODULI UI (invariati) ===
#include "src/ui_config.h"
#include "src/display_utils.h"
#include "src/touch_handler.h"
#include "src/menu_state.h"
#include "src/service_menu.h"
#include "src/leaf_actions.h"

// === NUOVI HANDLER SENSORI ===
#include "src/om60_handler.h"     // Nuovo sensore distanza
#include "src/hwt906_handler.h"   // Nuovo IMU

// === CONFIGURAZIONE PIN DISPLAY ===
#define LCD_CS      42
#define LCD_RST     39
#define LCD_DC      41
#define LCD_SCK     40
#define LCD_MOSI    45
#define LCD_BL      5

// === CONFIGURAZIONE PIN TOUCH ===
#define TP_SDA      1      
#define TP_SCL      3      

// === CONFIGURAZIONE PIN I2C SENSORI ===
#define IMU_SDA     11    // I2C per configurazione HWT906
#define IMU_SCL     10

// === OGGETTI HARDWARE ===
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true);

CSE_CST328 touchObj(240, 320, &Wire1);
CSE_CST328 *touch = &touchObj;

// === CONFIGURAZIONE UI ===
UIConfig ui;

// === VARIABILI GLOBALI ===
bool sensorsInitialized = false;
uint32_t lastSensorUpdate = 0;
uint32_t lastDisplayUpdate = 0;
const uint32_t SENSOR_UPDATE_INTERVAL = 100;  // 10Hz
const uint32_t DISPLAY_UPDATE_INTERVAL = 100; // 10Hz

// === FUNZIONE ESTERNA DAL MODULO display_utils ===
extern void drawMenu(Arduino_GFX* gfx, UIConfig& ui, MenuState state);

// ========================================
// SETUP
// ========================================
void setup() {
    // === INIZIALIZZAZIONE SERIALE ===
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n================================");
    Serial.println("ESP32 HySeq OM60-HWT906 v2.0");
    Serial.println("================================");
    Serial.println("Sistema di misura professionale");
    Serial.println("OM60: Laser 200-1000mm");
    Serial.println("HWT906: IMU 9-assi");
    Serial.println("================================\n");
    
    // === INIZIALIZZAZIONE HARDWARE ===
    Serial.println("🔧 Inizializzazione hardware...");
    
    // Backlight
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    
    // I2C buses
    Serial.println("📡 Configurazione I2C...");
    Wire.begin(IMU_SDA, IMU_SCL);     // Bus principale per sensori
    Wire.setClock(100000);            // 100kHz standard
    Wire1.begin(TP_SDA, TP_SCL);      // Bus secondario per touch
    Wire1.setClock(100000);
    
    // Display
    Serial.println("🖥️ Inizializzazione display...");
    gfx->begin();
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);
    
    // Touch
    Serial.println("👆 Inizializzazione touch...");
    touch->begin();
    
    // EEPROM - rimosso e sostituito da Preferences.h
    // Serial.println("💾 Inizializzazione EEPROM...");
    // EEPROM.begin(512);
    
    // === CONFIGURAZIONE UI ===
    ui.topLine = (char*)"HySeq OM60";
    ui.botLine = (char*)"by Picobarn - v2.0";
    ui.bgColor = RGB565_BLUE;
    ui.textColor = WHITE;
    // ui.highlightColor = YELLOW;
    
    // === SCHERMATA DI AVVIO ===
    showStartupScreen();
    
    // === INIZIALIZZAZIONE SENSORI ===
   #ifdef USE_FREERTOS
    if (!initSensorTasks()) {
        Serial.println("Errore init FreeRTOS tasks!");
    }
   #endif
   
    Serial.println("\n📡 Inizializzazione sensori...");
    
    // HWT906 (IMU)
    bool hwt906_ok = initHWT906();
    if (isHWT906Present()) {
        gfx->setCursor(20, 140);
        gfx->setTextColor(GREEN);
        gfx->println("✓ HWT906 IMU OK");
        Serial.println("✅ HWT906 inizializzato correttamente");
        
        // Configura per uso statico se presente
        configureHWT906Mode(HWT906_MODE_STATIC_MED);
        enableHWT906DriftCompensation(true);
    } else {
        gfx->setCursor(20, 140);
        gfx->setTextColor(ORANGE);
        gfx->println("⚠ HWT906 non trovato");
        Serial.println("⚠️ HWT906 non trovato - continuo senza IMU");
    }
    
    // OM60 (Distanza) - Per ora simulato
    bool om60_ok = false;  // initOM60() quando disponibile
    gfx->setCursor(20, 160);
    gfx->setTextColor(DARKGREY);
    gfx->println("○ OM60 non implementato");
    Serial.println("ℹ️ OM60 in attesa di implementazione");
    
    // Attendi prima di procedere
    delay(2000);
    
    // === VERIFICA FINALE ===
    sensorsInitialized = hwt906_ok || om60_ok;
    if (!sensorsInitialized) {
        Serial.println("\n⚠️ ATTENZIONE: Nessun sensore trovato!");
        Serial.println("Il sistema continuerà in modalità demo");
        
        gfx->fillScreen(BLACK);
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        gfx->setCursor(40, 100);
        gfx->println("MODO DEMO");
        gfx->setTextSize(1);
        gfx->setCursor(30, 140);
        gfx->println("Nessun sensore rilevato");
        delay(2000);
    }
    
    // === MENU PRINCIPALE ===
    Serial.println("\n✅ Sistema pronto!");
    drawMainMenu(gfx, ui);
    
    Serial.println("\n=== READY FOR OPERATION ===\n");
}

// ========================================
// LOOP PRINCIPALE
// ========================================
void loop() {
    uint32_t currentTime = millis();
    
// === UPDATE SENSORI (10Hz) ===
    #ifdef USE_FREERTOS
        // Con FreeRTOS i sensori sono aggiornati automaticamente dal task
        // Non serve fare nulla qui, i dati sono sempre pronti
        // Puoi eventualmente verificare che il task sia attivo:
        // if (sensorTaskHandle == NULL) {
        //     Serial.println("WARNING: Sensor task not running!");
        // }
    #else
        // Modalità normale - update manuale
        if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
            lastSensorUpdate = currentTime;
            
            // Update HWT906 se presente
            if (isHWT906Present()) {
                updateHWT906();
            }
            
            // Update OM60 quando implementato
            // if (isOM60Present()) {
            //     updateOM60();
            // }
        }
    #endif
    
    // === GESTIONE TOUCH ===
    handleTouch(gfx, ui);
    
    // === UPDATE DISPLAY SE IN LIVE MODE ===
    MenuState currentState = getCurrentMenuState();
    if (currentState == DISPLAY_LIVE_DATA) {
        if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
            lastDisplayUpdate = currentTime;
            // L'update è gestito da leaf_actions.cpp
        }
    }
    
    // === DIAGNOSTICA PERIODICA ===
    static uint32_t lastDiagnostic = 0;
    if (currentTime - lastDiagnostic > 30000) {  // Ogni 30 secondi
        lastDiagnostic = currentTime;
        printSystemStatus();
    }
    
    // === CHECK RICONNESSIONE SENSORI ===
    static uint32_t lastReconnectCheck = 0;
    if (!sensorsInitialized && currentTime - lastReconnectCheck > 5000) {
        lastReconnectCheck = currentTime;
        checkSensorReconnection();
    }
    
    // === COMANDI SERIALI ===
    if (Serial.available()) {
        handleSerialCommands();
    }
    
    // Piccolo delay per non sovraccaricare
    delay(10);
}

// ========================================
// FUNZIONI DI SUPPORTO
// ========================================

void showStartupScreen() {
    gfx->fillScreen(RGB565_BLUE);
    
    // Logo/Titolo
    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(45, 30);
    gfx->println("HySeq");
    
    gfx->setTextSize(1);
    gfx->setCursor(65, 60);
    gfx->println("OM60 + HWT906");
    
    // Info sistema
    gfx->setTextColor(CYAN);
    gfx->setCursor(20, 100);
    gfx->println("Inizializzazione sistema...");
    
    // Barra di progresso fittizia
    gfx->drawRect(20, 120, 200, 10, WHITE);
    for (int i = 0; i < 200; i += 20) {
        gfx->fillRect(20, 120, i, 10, GREEN);
        delay(50);
    }
}

void checkSensorReconnection() {
    bool reconnected = false;
    
    // Check HWT906
    if (!isHWT906Present()) {
        if (initHWT906()) {
            Serial.println("✅ HWT906 riconnesso!");
            reconnected = true;
        }
    }
    
    // Check OM60 quando implementato
    // if (!isOM60Present()) {
    //     if (initOM60()) {
    //         Serial.println("✅ OM60 riconnesso!");
    //         reconnected = true;
    //     }
    // }
    
    if (reconnected) {
        sensorsInitialized = true;
    }
}

void printSystemStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
    Serial.printf("Free heap: %lu bytes\n", ESP.getFreeHeap());
    Serial.printf("CPU freq: %lu MHz\n", ESP.getCpuFreqMHz());
    
    // Stato sensori
    Serial.println("\nSensori:");
    if (isHWT906Present()) {
        Serial.printf("  HWT906: OK (%.1f Hz)\n", getHWT906UpdateRate());
        Serial.printf("    Pitch: %.1f°  Yaw: %.1f°\n", 
                      getHWT906Pitch(), getHWT906Yaw());
    } else {
        Serial.println("  HWT906: NON PRESENTE");
    }
    
    // OM60 quando implementato
    Serial.println("  OM60: NON IMPLEMENTATO");
    
    Serial.println("====================\n");
}

void handleSerialCommands() {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "STATUS") {
        printSystemStatus();
    } 
    else if (cmd == "CAL") {
        Serial.println("Avvio calibrazione HWT906...");
        calibrateHWT906();
    }
    else if (cmd == "ZERO") {
        Serial.println("Azzeramento angoli HWT906...");
        zeroHWT906Angles();
    }
    else if (cmd == "DEBUG") {
        printHWT906Debug();
    }
    else if (cmd == "HELP") {
        Serial.println("\n=== COMANDI DISPONIBILI ===");
        Serial.println("STATUS - Mostra stato sistema");
        Serial.println("CAL    - Calibra HWT906");
        Serial.println("ZERO   - Azzera angoli");
        Serial.println("DEBUG  - Debug dettagliato HWT906");
        Serial.println("HELP   - Mostra questo menu");
        Serial.println("========================\n");
    }
    else {
        Serial.println("Comando non riconosciuto. Digita HELP");
    }
}



// ========================================
// END OF FILE
// ========================================