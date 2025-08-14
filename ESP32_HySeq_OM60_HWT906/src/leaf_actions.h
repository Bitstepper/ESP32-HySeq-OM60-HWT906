// leaf_actions.h
#ifndef LEAF_ACTIONS_H
#define LEAF_ACTIONS_H

#include <Arduino.h>

// Namespace per organizzare le azioni
namespace LeafActions {
    
// === SUBMENU 1: PITCH,YAW,DIST ===
void showHWT906Status();        // Era startAcquisition()
void showLiveData();           // INVARIATO
void showLiveGraph();          // INVARIATO  
void showHWT906Config();       // Era exportCSV
void showHWT906Calib(); 
    
    // === SUBMENU 2: CALIB. IMU ===
    void calibrateGyro();
    void calibrateAccel();
    void calibrateMag();
    
    // === SUBMENU 3: SERVICE ===
    void changeSettings();
    void showSystemInfo();
    void factoryReset();
    
    // === UTILITY ===
    void stopCurrentAction();
    bool isActionRunning();
    void updateDiagnosticMenu();    // ← AGGIUNTA
}

#endif // LEAF_ACTIONS_H
