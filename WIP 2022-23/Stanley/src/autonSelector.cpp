#include "vex.h"
using namespace vex;
#include <string>
class button {
    
    private:
    int x;
    int y;
    int height;
    int width;
    vex::color boxColor;
    std::string text;
    public:
    button(int x, int y, int height, int width, vex::color boxColor, std::string text) : x(x),y(y),height(height), width(width), boxColor(boxColor), text(text){
        Brain.Screen.drawRectangle(x,y,width,height,boxColor);
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
};
