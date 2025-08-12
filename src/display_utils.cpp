// display_utils.cpp
#include "display_utils.h"
#include "ui_config.h"

// Definizione puntatori globali
extern Arduino_GFX* gfx;
extern CSE_CST328 *touch;
extern UIConfig ui;

// display_utils.cpp - Aggiungi questa implementazione
void DisplayUtils::drawRect(int x, int y, int w, int h, uint16_t color) {
    gfx->drawRect(x, y, w, h, color);
}

DisplayUtils::DisplayUtils() {
    // I puntatori globali sono già inizializzati nel main
}

void DisplayUtils::clearScreen() {
    // Usa direttamente il colore senza controllare ui
    gfx->fillScreen(BLACK);  // Schermo nero
    delay(100);  // Piccolo delay
}

void DisplayUtils::drawHeader() {
    // Versione semplificata per test
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(50, 50);
    gfx->println("HEADER TEST");
}

void DisplayUtils::drawFooter() {
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->setCursor(35, 310);
    gfx->println(ui.botLine);
}

void DisplayUtils::setText(int x, int y, const char* text, uint16_t color, uint8_t size) {
    gfx->setTextColor(color);
    gfx->setTextSize(size);
    gfx->setCursor(x, y);
    gfx->println(text);
}

void DisplayUtils::setTextCentered(int y, const char* text, uint16_t color, uint8_t size) {
    gfx->setTextColor(color);
    gfx->setTextSize(size);
    
    // Calcola larghezza testo approssimativa
    int textWidth = strlen(text) * 6 * size;
    int x = (240 - textWidth) / 2;
    
    gfx->setCursor(x, y);
    gfx->println(text);
}

void DisplayUtils::drawBox(int x, int y, int w, int h, uint16_t color) {
    gfx->drawRect(x, y, w, h, color);
}

void DisplayUtils::fillRect(int x, int y, int w, int h, uint16_t color) {
    gfx->fillRect(x, y, w, h, color);
}

void DisplayUtils::drawButton(int x, int y, int w, int h, uint16_t color, const char* label) {
    gfx->fillRect(x, y, w, h, color);
    gfx->drawRect(x, y, w, h, WHITE);
    
    // Centra il testo nel pulsante
    int textWidth = strlen(label) * 6 * 2;
    int textX = x + (w - textWidth) / 2;
    int textY = y + (h - 16) / 2;
    
    setText(textX, textY, label, WHITE, 2);
}

void DisplayUtils::drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    gfx->drawLine(x0, y0, x1, y1, color);
}

void DisplayUtils::fillCircle(int x, int y, int r, uint16_t color) {
    gfx->fillCircle(x, y, r, color);
}

bool DisplayUtils::isTouchInArea(int x, int y, int w, int h) {
    if (touch->getTouches() == 0) return false; // Usa -> invece di .
    
    auto p = touch->touchPoints[0]; // Usa -> invece di .
    return (p.x >= x && p.x <= (x + w) && p.y >= y && p.y <= (y + h));
}

void DisplayUtils::showMessage(const char* title, const char* message, int duration) {
    clearScreen();
    drawHeader();
    
    setTextCentered(120, title, WHITE, 2);
    setTextCentered(160, message, YELLOW, 1);
    
    delay(duration);
}
