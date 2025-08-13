// display_utils.h - VERSIONE CORRETTA
#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include <Arduino_GFX_Library.h>
#include <CSE_CST328.h>
#include "ui_config.h"

class DisplayUtils {
private:
    Arduino_GFX* gfx;
    CSE_CST328* touch;  // Puntatore, non oggetto
    
public:
    DisplayUtils();
    
    // Funzioni base display
    void clearScreen();
    void drawHeader();
    void drawFooter();
    
    // Funzioni testo
    void setText(int x, int y, const char* text, uint16_t color, uint8_t size);
    void setTextCentered(int y, const char* text, uint16_t color, uint8_t size);
    
    // Funzioni grafiche
    void drawBox(int x, int y, int w, int h, uint16_t color);
    void drawRect(int x, int y, int w, int h, uint16_t color);  // AGGIUNTO
    void fillRect(int x, int y, int w, int h, uint16_t color);
    void drawButton(int x, int y, int w, int h, uint16_t color, const char* label);
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color);
    void fillCircle(int x, int y, int r, uint16_t color);
    
    // Funzioni touch
    bool isTouchInArea(int x, int y, int w, int h);
    
    // Utility
    void showMessage(const char* title, const char* message, int duration);
};

// Dichiarazione oggetti globali esterni
extern Arduino_GFX* gfx;
extern CSE_CST328 *touch;

#endif // DISPLAY_UTILS_H
