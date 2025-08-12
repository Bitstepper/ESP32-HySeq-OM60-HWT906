// touch_handler.h
#ifndef TOUCH_HANDLER_H
#define TOUCH_HANDLER_H

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <CSE_CST328.h>
#include "ui_config.h"
#include "menu_state.h"

// === FUNZIONI PRINCIPALI ===
// Gestione touch principale
void handleTouch(Arduino_GFX* gfx, UIConfig& ui);

// Disegno menu
void drawMainMenu(Arduino_GFX* gfx, UIConfig& ui);
void drawMenu(Arduino_GFX* gfx, UIConfig& ui, MenuState state);

// === GESTIONE SELEZIONI ===
void handleMenuConfirm(Arduino_GFX* gfx, UIConfig& ui);
void handleMainMenuSelection(int index, Arduino_GFX* gfx, UIConfig& ui);
void handleSubmenu1Selection(int index, Arduino_GFX* gfx, UIConfig& ui);
void handleSubmenu2Selection(int index, Arduino_GFX* gfx, UIConfig& ui);
void handleSubmenu3Selection(int index, Arduino_GFX* gfx, UIConfig& ui);

// === GESTIONE STATI SPECIALI ===
void handleLiveDataTouch(int x, int y, Arduino_GFX* gfx, UIConfig& ui);
void handleCalibrationTouch(int x, int y, Arduino_GFX* gfx, UIConfig& ui);

// === UTILITY ===
void showMessage(Arduino_GFX* gfx, const char* msg, uint16_t textColor, uint16_t bgColor);
bool confirmDialog(Arduino_GFX* gfx, const char* title, const char* message);

// === LEGACY (per compatibilità) ===
void checkTouch();

#endif // TOUCH_HANDLER_H
