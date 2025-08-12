// menu_state.cpp
#include "menu_state.h"

// === DEFINIZIONI DEGLI ARRAY (una sola volta qui!) ===
const char* mainMenuItems[] = {
    "PITCH,YAW,DIST",
    "CALIB. IMU", 
    "SERVICE MENU"
};

const char* submenu1Items[] = {
    "Start Acquis.",
    "Live Graph",
    "Export CSV"
};

const char* submenu2Items[] = {
    "Gyro Calib",
    "Accel Calib",
    "Mag Calib"
};

const char* submenu3Items[] = {
    "Settings",
    "About",
    "Factory Reset"
};


// === VARIABILI GLOBALI DI STATO ===
MenuState currentMenuState = MAIN_MENU;
int selectedMenuIndex = -1;
MenuState previousMenuState = MAIN_MENU;

// Stack per navigazione (max 5 livelli)
static MenuState navigationStack[5];
static int stackPointer = 0;

// === FUNZIONI BASE ===
MenuState getCurrentMenuState() {
    return currentMenuState;
}

void setCurrentMenuState(MenuState state) {
    currentMenuState = state;
}

// === NAVIGAZIONE ===
void navigateToMenu(MenuState state) {
    // Salva stato corrente nello stack
    if (stackPointer < 5) {
        navigationStack[stackPointer++] = currentMenuState;
    }
    
    previousMenuState = currentMenuState;
    currentMenuState = state;
    selectedMenuIndex = -1;  // Reset selezione
    
    Serial.printf("Navigate: %s -> %s\n", 
                  getMenuTitle(previousMenuState), 
                  getMenuTitle(state));
}

void navigateBack() {
    if (canNavigateBack()) {
        // Ripristina stato precedente dallo stack
        currentMenuState = navigationStack[--stackPointer];
        selectedMenuIndex = -1;
        
        Serial.printf("Navigate back to: %s\n", getMenuTitle(currentMenuState));
    }
}

bool canNavigateBack() {
    return stackPointer > 0 && currentMenuState != MAIN_MENU;
}

// === SELEZIONE ===
int getSelectedIndex() {
    return selectedMenuIndex;
}

void setSelectedIndex(int index) {
    int maxItems = getMenuItemCount(currentMenuState);
    if (index >= -1 && index < maxItems) {
        selectedMenuIndex = index;
    }
}

void resetSelectedIndex() {
    selectedMenuIndex = -1;
}

// === UTILITY ===
const char* getMenuTitle(MenuState state) {
    switch (state) {
        case MAIN_MENU:         return "MAIN MENU";
        case SUBMENU_1:         return "PITCH,YAW,DIST";
        case SUBMENU_2:         return "CALIB. IMU";
        case SUBMENU_3:         return "SERVICE MENU";
        case DISPLAY_LIVE_DATA: return "LIVE DATA";
        case CALIBRATING_IMU:   return "CALIBRATING...";
        case RUNNING_TEST:      return "RUNNING TEST...";
        case ERROR_STATE:       return "ERROR";
        default:                return "UNKNOWN";
    }
}

const char** getMenuItems(MenuState state) {
    switch (state) {
        case MAIN_MENU:  return mainMenuItems;
        case SUBMENU_1:  return submenu1Items;
        case SUBMENU_2:  return submenu2Items;
        case SUBMENU_3:  return submenu3Items;
        default:         return nullptr;
    }
}

int getMenuItemCount(MenuState state) {
    switch (state) {
        case MAIN_MENU:
        case SUBMENU_1:
        case SUBMENU_2:
        case SUBMENU_3:
            return SUBMENU_ITEMS;
        default:
            return 0;
    }
}

const char* getCurrentMenuItemText() {
    const char** items = getMenuItems(currentMenuState);
    if (items && selectedMenuIndex >= 0 && selectedMenuIndex < getMenuItemCount(currentMenuState)) {
        return items[selectedMenuIndex];
    }
    return "No Selection";
}

// === STATI SPECIALI ===
bool isInLiveDataMode() {
    return currentMenuState == DISPLAY_LIVE_DATA;
}

bool isCalibrating() {
    return currentMenuState == CALIBRATING_IMU;
}

bool needsServicePIN() {
    // Richiede PIN quando si seleziona SERVICE MENU dal main menu
    return currentMenuState == MAIN_MENU && 
           selectedMenuIndex == 2;  // Terza voce = SERVICE MENU
}

// === DEBUG ===
void printMenuDebug() {
    Serial.println("\n=== Menu State Debug ===");
    Serial.printf("Current State: %s\n", getMenuTitle(currentMenuState));
    Serial.printf("Selected Index: %d\n", selectedMenuIndex);
    Serial.printf("Navigation Stack Depth: %d\n", stackPointer);
    
    if (selectedMenuIndex >= 0) {
        Serial.printf("Selected Item: %s\n", getCurrentMenuItemText());
    }
    
    Serial.println("Stack:");
    for (int i = 0; i < stackPointer; i++) {
        Serial.printf("  [%d] %s\n", i, getMenuTitle(navigationStack[i]));
    }
    Serial.println("=====================\n");
}
