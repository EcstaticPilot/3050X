#pragma once
#include "vex.h"

class button {
    
    private:
    int x;
    int y;
    int height;
    int width;
    vex::color boxColor;
    std::string text;
    bool isFilled;

    public:
    /**
     * @brief Construct a new button object
     * 
     * @param x 
     * @param y 
     * @param height 
     * @param width 
     * @param boxColor 
     * @param text 
     */
    button(int x, int y, int height, int width, vex::color boxColor, std::string text);

    button(){}; //default constructor

    bool checkTouch();

    void setFill(bool fill);
};
void drawField();


