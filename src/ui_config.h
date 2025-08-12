// ui_config.h - File di configurazione UI
#ifndef UI_CONFIG_H
#define UI_CONFIG_H

#include <Arduino.h>  // Solo per i tipi uint16_t, uint8_t, etc.

// === COLORI RGB565 ===
// Definizione colori base
#define BLACK     0x0000
#define WHITE     0xFFFF
#define RED       0xF800
#define GREEN     0x07E0
#define BLUE      0x001F
#define YELLOW    0xFFE0
#define ORANGE    0xFD20
#define LIGHTBLUE 0xAEDC
#define DARKGREY  0x7BEF

// Colori aggiuntivi personalizzati
#define DARKGREEN   0x03E0
#define DARKBLUE    0x0010
#define LIGHTRED    0xFBB2
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define BROWN       0xA145
#define PURPLE      0x780F

// Alias per compatibilità con codice esistente
#define RGB565_BLUE   BLUE
#define RGB565_YELLOW YELLOW

// === PIN CONFIGURAZIONE HARDWARE ===
// Pin per display ST7789
#define LCD_CS   42    // Chip Select
#define LCD_RST  39    // Reset
#define LCD_DC   41    // Data/Command
#define LCD_SCK  40    // Clock SPI
#define LCD_MOSI 45    // Data SPI
#define LCD_BL   5     // Backlight

// Pin per touch CST328
#define TP_SDA   1     // I2C Data (Wire1)
#define TP_SCL   3     // I2C Clock (Wire1)

// === PARAMETRI LAYOUT MENU ===
// Posizionamento pulsanti menu
#define BTN_X_OFFSET   20     // Margine sinistro
#define BTN_Y_START    60     // Posizione Y primo pulsante
#define BTN_SPACING    60     // Spazio verticale tra pulsanti
#define BTN_WIDTH      200    // Larghezza pulsanti menu
#define BTN_HEIGHT     50     // Altezza pulsanti menu

// Posizionamento pulsanti azione (Back/Confirm)
#define BTN_BACK_X     BTN_X_OFFSET
#define BTN_CONFIRM_X  (BTN_BACK_X + 90 + 20)  // Back width + gap
#define BTN_Y_BOTTOM   260    // Posizione Y pulsanti bottom

// === DIMENSIONI DISPLAY ===
#define SCREEN_WIDTH   240
#define SCREEN_HEIGHT  320

// === STRUTTURA CONFIGURAZIONE UI ===
// Questa struttura contiene tutte le impostazioni modificabili dell'UI
struct UIConfig {
    // Colori tema
    uint16_t bgColor = LIGHTBLUE;       // Colore sfondo
    uint16_t textColor = YELLOW;        // Colore testo normale
    uint16_t buttonColor = DARKGREY;    // Colore pulsanti inattivi
    uint16_t activeColor = BLACK;       // Colore pulsante selezionato
    
    // Dimensioni testo
    uint8_t textSize = 2;               // Dimensione testo menu
    uint8_t titleSize = 3;              // Dimensione titoli
    uint8_t smallTextSize = 1;          // Dimensione testo piccolo
    
    // Stringhe personalizzabili
    char* topLine = nullptr;            // Intestazione display
    char* botLine = nullptr;            // Piede pagina display
    
    // Timing
    uint16_t debounceDelay = 200;      // Delay anti-rimbalzo touch
    uint16_t refreshRate = 100;         // Refresh rate display live data
};

// === COSTANTI MENU ===
// Numero massimo di voci per menu
#define MAX_MENU_ITEMS 3

// === SERVICE CODE ===
// PIN per accesso menu service
#define SERVICE_PIN "235711"

#endif // UI_CONFIG_H
