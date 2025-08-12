// menu_state.h
#ifndef MENU_STATE_H
#define MENU_STATE_H

#include <Arduino.h>

// === ENUM STATI MENU ===
enum MenuState {
    MAIN_MENU,
    SUBMENU_1,        // PITCH,YAW,DIST submenu
    SUBMENU_2,        // CALIB. IMU submenu
    SUBMENU_3,        // SERVICE MENU submenu
    DISPLAY_LIVE_DATA,
    CALIBRATING_IMU,
    RUNNING_TEST,
    ERROR_STATE
};

// === VOCI DI MENU (dal progetto originale) ===
// === DICHIARAZIONI EXTERN DEGLI ARRAY (non definizioni!) ===
extern const char* mainMenuItems[];
extern const char* submenu1Items[];
extern const char* submenu2Items[];
extern const char* submenu3Items[];

// === CONFIGURAZIONE MENU ===
#define MAIN_MENU_ITEMS 3
#define SUBMENU_ITEMS 3
#define SERVICE_PIN "235711"

// === VARIABILI GLOBALI DI STATO ===
extern MenuState currentMenuState;
extern int selectedMenuIndex;
extern MenuState previousMenuState;  // Per tornare indietro

// === FUNZIONI DI GESTIONE STATO ===
// Getter/Setter base
MenuState getCurrentMenuState();
void setCurrentMenuState(MenuState state);

// Navigazione
void navigateToMenu(MenuState state);
void navigateBack();
bool canNavigateBack();

// Selezione
int getSelectedIndex();
void setSelectedIndex(int index);
void resetSelectedIndex();

// Utility
const char* getMenuTitle(MenuState state);
const char** getMenuItems(MenuState state);
int getMenuItemCount(MenuState state);
const char* getCurrentMenuItemText();

// Azioni specifiche
bool isInLiveDataMode();
bool isCalibrating();
bool needsServicePIN();

// Debug
void printMenuDebug();

#endif // MENU_STATE_H
