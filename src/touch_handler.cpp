// touch_handler.cpp
#include "touch_handler.h"
#include "menu_state.h"
#include "display_utils.h"
#include "leaf_actions.h"
#include "service_menu.h"
#include "ui_config.h"

// Puntatori esterni
extern CSE_CST328 *touch;
extern Arduino_GFX *gfx;

// Debounce
static uint32_t lastTouchTime = 0;
static const uint32_t TOUCH_DEBOUNCE_MS = 200;

// === DISEGNO MENU PRINCIPALE ===
void drawMainMenu(Arduino_GFX* gfx, UIConfig& ui) {
    drawMenu(gfx, ui, MAIN_MENU);
}

// === DISEGNO MENU GENERICO ===
void drawMenu(Arduino_GFX* gfx, UIConfig& ui, MenuState state) {
    gfx->fillScreen(ui.bgColor);
    
    // === HEADER ===
    if (state == MAIN_MENU) {
        // Header completo solo per main menu
        gfx->setTextColor(DARKBLUE);
        gfx->setTextSize(3);
        gfx->setCursor(30, 10);
        gfx->println(ui.topLine);
        
        // Footer
        gfx->setTextSize(1);
        gfx->setCursor(35, 310);
        gfx->setTextColor(WHITE);
        gfx->println(ui.botLine);
    } else {
        // Per submenu mostra il titolo
        gfx->setTextColor(ui.textColor);
        gfx->setTextSize(2);
        gfx->setCursor(20, 20);
        gfx->println(getMenuTitle(state));
    }
    
    // === DISEGNA VOCI MENU ===
    const char** items = getMenuItems(state);
    int itemCount = getMenuItemCount(state);
    int selectedIndex = getSelectedIndex();
    
    if (items && itemCount > 0) {
        gfx->setTextSize(ui.textSize);
        
        for (int i = 0; i < itemCount; i++) {
            int y = BTN_Y_START + i * BTN_SPACING;
            
            // Colore basato su selezione
            uint16_t bgColor = (i == selectedIndex) ? ui.activeColor : ui.buttonColor;
            uint16_t textColor = (i == selectedIndex) ? WHITE : ui.textColor;
            
            // Disegna pulsante
            gfx->fillRect(BTN_X_OFFSET, y, BTN_WIDTH, BTN_HEIGHT, bgColor);
            gfx->drawRect(BTN_X_OFFSET, y, BTN_WIDTH, BTN_HEIGHT, WHITE);
            
            // Testo centrato verticalmente
            gfx->setCursor(BTN_X_OFFSET + 20, y + (BTN_HEIGHT - 16) / 2);
            gfx->setTextColor(textColor);
            gfx->println(items[i]);
        }
    }
    
    // === PULSANTI NAVIGAZIONE ===
    // BACK button (solo se non siamo nel main menu)
    if (canNavigateBack()) {
        gfx->fillRect(BTN_BACK_X, BTN_Y_BOTTOM, 100, 40, ORANGE);
        gfx->drawRect(BTN_BACK_X, BTN_Y_BOTTOM, 100, 40, BLACK);
        gfx->setCursor(BTN_BACK_X + 25, BTN_Y_BOTTOM + 12);
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->println("BACK");
    }
    
    // OK/CONFIRM button (sempre visibile, abilitato solo con selezione)
    uint16_t confirmColor = (selectedIndex >= 0) ? DARKGREEN : DARKGREY;
    gfx->fillRect(BTN_CONFIRM_X, BTN_Y_BOTTOM, 90, 40, confirmColor);
    gfx->drawRect(BTN_CONFIRM_X, BTN_Y_BOTTOM, 90, 40, BLACK);
    gfx->setCursor(BTN_CONFIRM_X + 25, BTN_Y_BOTTOM + 12);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->println("OK");
}

// === GESTIONE TOUCH PRINCIPALE ===
void handleTouch(Arduino_GFX* gfx, UIConfig& ui) {
    // Verifica touch disponibile
    if (touch == nullptr || touch->getTouches() == 0) return;
    
    // Debounce
    if (millis() - lastTouchTime < TOUCH_DEBOUNCE_MS) return;
    lastTouchTime = millis();
    
    // Ottieni coordinate
    auto p = touch->touchPoints[0];
    int x = p.x;
    int y = p.y;
    
    // Debug coordinate (opzionale)
    Serial.printf("Touch: x=%d, y=%d\n", x, y);
    
    // Gestione basata sullo stato corrente
    MenuState currentState = getCurrentMenuState();
    
    // === GESTIONE STATI SPECIALI ===
    if (isInLiveDataMode()) {
        handleLiveDataTouch(x, y, gfx, ui);
        return;
    }
    
    if (isCalibrating()) {
        // Durante calibrazione, ignora touch o mostra solo "Cancel"
        handleCalibrationTouch(x, y, gfx, ui);
        return;
    }
    
    // === GESTIONE MENU NORMALE ===
    
    // Check BACK button
    if (x >= BTN_BACK_X && x <= (BTN_BACK_X + 100) && 
        y >= BTN_Y_BOTTOM && y <= (BTN_Y_BOTTOM + 40)) {
        
        if (canNavigateBack()) {
            navigateBack();
            drawMenu(gfx, ui, getCurrentMenuState());
        }
        return;
    }
    
    // Check OK/CONFIRM button
    if (x >= BTN_CONFIRM_X && x <= (BTN_CONFIRM_X + 90) && 
        y >= BTN_Y_BOTTOM && y <= (BTN_Y_BOTTOM + 40)) {
        
        if (getSelectedIndex() >= 0) {
            handleMenuConfirm(gfx, ui);
        }
        return;
    }
    
    // Check menu items
    int itemCount = getMenuItemCount(currentState);
    for (int i = 0; i < itemCount; i++) {
        int yTop = BTN_Y_START + i * BTN_SPACING;
        int yBottom = yTop + BTN_HEIGHT;
        
        if (x >= BTN_X_OFFSET && x <= (BTN_X_OFFSET + BTN_WIDTH) && 
            y >= yTop && y <= yBottom) {
            
            setSelectedIndex(i);
            drawMenu(gfx, ui, currentState);
            return;
        }
    }
}

// === GESTIONE CONFERMA MENU ===
void handleMenuConfirm(Arduino_GFX* gfx, UIConfig& ui) {
    MenuState currentState = getCurrentMenuState();
    int selectedIndex = getSelectedIndex();
    
    switch (currentState) {
        case MAIN_MENU:
            handleMainMenuSelection(selectedIndex, gfx, ui);
            break;
            
        case SUBMENU_1:
            handleSubmenu1Selection(selectedIndex, gfx, ui);
            break;
            
        case SUBMENU_2:
            handleSubmenu2Selection(selectedIndex, gfx, ui);
            break;
            
        case SUBMENU_3:
            handleSubmenu3Selection(selectedIndex, gfx, ui);
            break;
            
        default:
            break;
    }
}

// === GESTIONE SELEZIONI MAIN MENU ===
void handleMainMenuSelection(int index, Arduino_GFX* gfx, UIConfig& ui) {
    switch (index) {
        case 0: // PITCH,YAW,DIST
            navigateToMenu(SUBMENU_1);
            break;
            
        case 1: // CALIB. IMU
            navigateToMenu(SUBMENU_2);
            break;
            
        case 2: // SERVICE MENU
            // Richiede PIN
            if (enterServiceCode(gfx, ui)) {
                navigateToMenu(SUBMENU_3);
            }
            return; // enterServiceCode ridisegna già il menu
    }
    
    drawMenu(gfx, ui, getCurrentMenuState());
}

// === GESTIONE SUBMENU 1 (PITCH,YAW,DIST) ===
void handleSubmenu1Selection(int index, Arduino_GFX* gfx, UIConfig& ui) {
    switch (index) {
        case 0: // Start Acquis.
            showMessage(gfx, "Starting...", WHITE, BLACK);
            LeafActions::startDataAcquisition();
            break;
            
        case 1: // Live Graph
            setCurrentMenuState(DISPLAY_LIVE_DATA);
            LeafActions::showLiveData();
            return; // Non ridisegnare menu
            
        case 2: // Export CSV
            showMessage(gfx, "Exporting...", WHITE, BLACK);
            LeafActions::exportCSV();
            break;
    }
    
    // Ridisegna menu dopo azione
    delay(1000);
    drawMenu(gfx, ui, getCurrentMenuState());
}

// === GESTIONE SUBMENU 2 (CALIB. IMU) ===
void handleSubmenu2Selection(int index, Arduino_GFX* gfx, UIConfig& ui) {
    switch (index) {
        case 0: // Gyro Calib
            setCurrentMenuState(CALIBRATING_IMU);
            LeafActions::calibrateGyro();
            break;
            
        case 1: // Accel Calib
            setCurrentMenuState(CALIBRATING_IMU);
            LeafActions::calibrateAccel();
            break;
            
        case 2: // Mag Calib
            setCurrentMenuState(CALIBRATING_IMU);
            LeafActions::calibrateMag();
            break;
    }
    
    // La calibrazione gestirà il proprio display
}

// === GESTIONE SUBMENU 3 (SERVICE) ===
void handleSubmenu3Selection(int index, Arduino_GFX* gfx, UIConfig& ui) {
    switch (index) {
        case 0: // Settings
            showMessage(gfx, "Settings...", WHITE, BLACK);
            LeafActions::changeSettings();
            break;
            
        case 1: // About
            LeafActions::showSystemInfo();
            break;
            
        case 2: // Factory Reset
            if (confirmDialog(gfx, "Factory Reset?", "This will erase all data!")) {
                showMessage(gfx, "Resetting...", RED, BLACK);
                LeafActions::factoryReset();
                delay(2000);
                ESP.restart();
            }
            break;
    }
    
    // Ridisegna menu dopo azione
    delay(1000);
    drawMenu(gfx, ui, getCurrentMenuState());
}

// === GESTIONE TOUCH LIVE DATA ===
void handleLiveDataTouch(int x, int y, Arduino_GFX* gfx, UIConfig& ui) {
    // Solo BACK button attivo
    if (x >= 20 && x <= 120 && y >= 280 && y <= 320) {
        setCurrentMenuState(SUBMENU_1);  // Torna al submenu
        drawMenu(gfx, ui, getCurrentMenuState());
    }
}

// === GESTIONE TOUCH CALIBRAZIONE ===
void handleCalibrationTouch(int x, int y, Arduino_GFX* gfx, UIConfig& ui) {
    // Pulsante CANCEL durante calibrazione
    if (x >= 70 && x <= 170 && y >= 250 && y <= 290) {
        // Interrompi calibrazione
        setCurrentMenuState(SUBMENU_2);
        showMessage(gfx, "Calibration Cancelled", YELLOW, BLACK);
        delay(1000);
        drawMenu(gfx, ui, getCurrentMenuState());
    }
}

// === FUNZIONI UTILITY ===
void showMessage(Arduino_GFX* gfx, const char* msg, uint16_t textColor, uint16_t bgColor) {
    gfx->fillScreen(bgColor);
    gfx->setTextColor(textColor);
    gfx->setTextSize(2);
    
    // Centra il testo
    int16_t x1, y1;
    uint16_t w, h;
    gfx->getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    
    int x = (240 - w) / 2;
    int y = (320 - h) / 2;
    
    gfx->setCursor(x, y);
    gfx->println(msg);
}

bool confirmDialog(Arduino_GFX* gfx, const char* title, const char* message) {
    // Disegna dialog
    gfx->fillScreen(BLACK);
    gfx->fillRect(20, 80, 200, 160, DARKGREY);
    gfx->drawRect(20, 80, 200, 160, WHITE);
    
    // Titolo
    gfx->setTextColor(YELLOW);
    gfx->setTextSize(2);
    gfx->setCursor(40, 100);
    gfx->println(title);
    
    // Messaggio
    gfx->setTextColor(WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(30, 130);
    gfx->println(message);
    
    // Pulsanti
    gfx->fillRect(40, 190, 60, 30, GREEN);
    gfx->drawRect(40, 190, 60, 30, WHITE);
    gfx->setCursor(55, 200);
    gfx->println("YES");
    
    gfx->fillRect(140, 190, 60, 30, RED);
    gfx->drawRect(140, 190, 60, 30, WHITE);
    gfx->setCursor(155, 200);
    gfx->println("NO");
    
    // Attendi risposta
    while (true) {
        if (touch->getTouches() > 0) {
            auto p = touch->touchPoints[0];
            
            // YES
            if (p.x >= 40 && p.x <= 100 && p.y >= 190 && p.y <= 220) {
                delay(200);
                return true;
            }
            
            // NO
            if (p.x >= 140 && p.x <= 200 && p.y >= 190 && p.y <= 220) {
                delay(200);
                return false;
            }
        }
        delay(50);
    }
}

// === FUNZIONE LEGACY (per compatibilità) ===
void checkTouch() {
    // Reindirizza a handleTouch con parametri globali
    if (gfx != nullptr) {
        UIConfig ui;  // Usa config default
        handleTouch(gfx, ui);
    }
}
