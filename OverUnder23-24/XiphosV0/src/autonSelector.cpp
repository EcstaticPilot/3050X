#include "vex.h"
#include "autonSelector.h"
using namespace vex;
#include <string>
    /**
     * @brief Construct a new button::button object
     * 
     * @param x 
     * @param y 
     * @param height 
     * @param width 
     * @param boxColor 
     * @param text 
     */
    button::button(int x, int y, int height, int width, vex::color boxColor, std::string text): x(x),y(y),height(height), width(width), boxColor(boxColor), text(text) {
        Brain.Screen.setPenColor(boxColor);
        Brain.Screen.drawRectangle(x,y,width,height);
        Brain.Screen.printAt(x+width/2,y+height/2,text.c_str());
    }
    /**
     * @brief checks if the button is being touched
     * 
     * @return true 
     * @return false 
     */
    bool button::checkTouch(){
       int xTouch =  Brain.Screen.xPosition();
       int yTouch =  Brain.Screen.yPosition();
       if((xTouch<=x+width&&xTouch>x)&&(yTouch<=y+height&&xTouch>y)){
        return true;
       } else{
        return false;
       }
    }
    /**
     * @brief sets the button to be filled or not
     * 
     * @param fill 
     */
    void button::setFill(bool fill){
        if(isFilled!=fill){
            isFilled = fill;
            if(isFilled){
                Brain.Screen.setFillColor(boxColor);
            } else {
                Brain.Screen.setPenColor(boxColor);
            }
            Brain.Screen.drawRectangle(x,y,width,height,boxColor);
            Brain.Screen.printAt(x+width/2,y+height/2,text.c_str());
        }
    }


void drawField(){

    color grey = color(132,132,132);
    /*
    Brain.Screen.setFillColor(grey);
    Brain.Screen.drawRectangle(130,30,227,227);
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(205,56,76,8);
    Brain.Screen.drawRectangle(239,58,8,170);
    Brain.Screen.drawRectangle(205,223,76,8);
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(129,86,38,113);
    Brain.Screen.drawRectangle(239,29,8,30);
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(318,86,38,114);
    Brain.Screen.drawRectangle(239,227,8,30);
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawCircle(239 + 4, 55 + 4, 4);
    Brain.Screen.drawCircle(239 + 4, 223 + 4 ,4);
    */

   //V2
    Brain.Screen.setFillColor(grey);
    Brain.Screen.drawRectangle(50,15,250,250);
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(132,54,87,5);
    Brain.Screen.drawRectangle(173,55,5,172);
    Brain.Screen.drawRectangle(132,222,86,5);
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(255,95,45,90);
    Brain.Screen.drawRectangle(173,227,5,38);
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(50,95,45,90);
    Brain.Screen.drawRectangle(173,15,4,40);
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawCircle(173 + 5, 53 + 5, 5);
    Brain.Screen.drawCircle(173 + 5, 222 + 5 , 5);
}