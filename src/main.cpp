#include <iostream>


#include "lib/SwitchControlLibrary.h"

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif

    auto& switchController = SwitchControlLibrary::getInstance();

    for (int i = 0; i < 10; i++) {
        switchController.delayTest();
    }

    std::string str;
    while (std::cin >> str) {
        if (str == "a") {
            switchController.pressButton(BUTTON_A);
        } else if (str == "A") {
            switchController.releaseButton(BUTTON_A);
        } else if (str == "b") {
            switchController.pressButton(BUTTON_B);
        } else if (str == "B") {
            switchController.releaseButton(BUTTON_B);
        } else if (str == "x") {
            switchController.pressButton(BUTTON_X);
        } else if(str == "X") {
            switchController.releaseButton(BUTTON_X);
        } else if (str == "y") {
            switchController.pressButton(BUTTON_Y);
        } else if (str == "Y") {
            switchController.releaseButton(BUTTON_Y);
        } else if (str == "l") {
            switchController.pressButton(BUTTON_L);
        } else if(str == "L") {
            switchController.releaseButton(BUTTON_L);
        } else if (str == "r") {
            switchController.pressButton(BUTTON_R);
        } else if (str == "R") {
            switchController.releaseButton(BUTTON_R);
        }
    }
}