#include "vex.h"
using namespace vex;
#include <string>

class button {
    
    private:
    int x;
    int y;
    int height;
    int width;
    bool isFilled = true;
    vex::color boxColor;
    std::string text;
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
    button(int x, int y, int height, int width, vex::color boxColor, std::string text) : x(x),y(y),height(height), width(width), boxColor(boxColor), text(text){
        Brain.Screen.setPenColor(boxColor);
        Brain.Screen.drawRectangle(x,y,width,height);
        Brain.Screen.printAt(x+width/2,y+height/2,text.c_str());
    }

    bool checkTouch(){
       int xTouch =  Brain.Screen.xPosition();
       int yTouch =  Brain.Screen.yPosition();
       if((xTouch<=x+width&&xTouch>x)&&(yTouch<=y+height&&xTouch>y)){
        return true;
       } else{
        return false;
       }
    }

    void setFill(bool fill){
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
};
