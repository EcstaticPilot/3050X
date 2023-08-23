#include "vex.h"
using namespace vex;
class button {
    
    public:
    int x;
    int y;
    int height;
    int width;
    vex::color boxColor;
    button(int x, int y, int height, int width, vex::color boxColor) : x(x),y(y),height(height), width(width), boxColor(boxColor){
        Brain.Screen.drawRectangle(x,y,width,height,boxColor);
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
};
