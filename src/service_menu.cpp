// service_menu.cpp
#include "service_menu.h"
#include "menu_state.h"
#include <CSE_CST328.h>

extern CSE_CST328 *touch;

bool enterServiceCode(Arduino_GFX* gfx, UIConfig& ui) {
    const char* serviceCode = SERVICE_PIN;  // "235711" da menu_state.h
    String input = "";

    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 10);
    gfx->println("=> Enter PIN:");

    // Tastiera numerica 3x4
    const char* keys[12] = { "1","2","3", "4","5","6", "7","8","9", "*","0","#" };
    
    // Disegna tastiera
    for (int i = 0; i < 12; i++) {
        int col = i % 3;
        int row = i / 3;
        int x = 40 + col * 70;
        int y = 60 + row * 50;
        gfx->fillRect(x, y, 60, 40, DARKGREY);
        gfx->drawRect(x, y, 60, 40, WHITE);
        gfx->setCursor(x + 20, y + 12);
        gfx->print(keys[i]);
    }

    while (true) {
        if (touch->getTouches()) {
            auto p = touch->touchPoints[0];
            int x = p.x, y = p.y;

            for (int i = 0; i < 12; i++) {
                int col = i % 3;
                int row = i / 3;
                int bx = 40 + col * 70;
                int by = 60 + row * 50;

                if (x >= bx && x <= bx + 60 && y >= by && y <= by + 40) {
                    const char* key = keys[i];

                    if (strcmp(key, "*") == 0) {
                        // Clear
                        input = "";
                        gfx->fillRect(40, 30, 200, 20, BLACK);
                    } else if (strcmp(key, "#") == 0) {
                        // Confirm
                        if (input == serviceCode) {
                            gfx->fillScreen(BLACK);
                            gfx->setCursor(20, 120);
                            gfx->setTextColor(GREEN);
                            gfx->println("[OK] PIN correct!");
                            delay(1500);
                            return true;
                        } else {
                            gfx->setCursor(20, 240);
                            gfx->setTextColor(RED);
                            gfx->println("[ERR] Wrong PIN!");
                            delay(1500);
                            input = "";
                            gfx->fillRect(40, 30, 200, 20, BLACK);
                            gfx->fillRect(20, 240, 200, 20, BLACK);
                        }
                    } else {
                        // Digit
                        if (input.length() < 6) {
                            input += key;
                            gfx->fillRect(40, 30, 200, 20, BLACK);
                            gfx->setCursor(40, 30);
                            gfx->setTextColor(YELLOW);
                            // Mostra asterischi
                            for (int j = 0; j < input.length(); j++) {
                                gfx->print("*");
                            }
                        }
                    }
                    
                    // Debounce
                    while (touch->getTouches()) delay(50);
                    break;
                }
            }
        }
        delay(50);
    }
}
