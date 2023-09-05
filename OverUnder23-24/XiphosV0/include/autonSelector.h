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


void drawField(){
    color grey = color(132,132,132);
    Brain.Screen.setFillColor(grey);
    Brain.Screen.drawRectangle(129.331,29.561,226.72,226.72);
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(204.992,56.018,75.591,7.559);
    Brain.Screen.drawRectangle(238.938,58.199,7.559,170.079);
    Brain.Screen.drawRectangle(204.992,223.410,75.591,7.559);
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(129.331,86.254,37.795,113.386);
    Brain.Screen.drawRectangle(238.938,29.312,7.559,30.236);
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(318.308,86.254,37.795,113.86);
    Brain.Screen.drawRectangle(238.938,226.561,7.559,30.236);
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawCircle(238.801 + 7.559/2, 54.504 + 7.559/2, 7.559/2);
    Brain.Screen.drawCircle(238.801 + 7.559/2, 222.635 + 7.559/2 ,7.559/2);
}

