/*******************************************************************************
 * Service Menu with PIN Protection
 ******************************************************************************/

#ifndef SERVICE_MENU_H
#define SERVICE_MENU_H

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "ui_config.h"

// Dichiarazione della funzione mancante
bool enterServiceCode(Arduino_GFX* gfx, UIConfig& ui);

// Altre funzioni service menu se necessarie
void showServiceMenu(Arduino_GFX* gfx, UIConfig& ui);

// PIN di accesso al menu service
#define SERVICE_PIN "235711"  // Usa #define invece di const char*

namespace ServiceMenu {
    bool authenticate();
    void showPinPad();
    bool checkPin(const String& enteredPin);
    void resetAttempts();
    
    extern int failedAttempts;
    extern const int MAX_ATTEMPTS;
}

#endif // SERVICE_MENU_H
