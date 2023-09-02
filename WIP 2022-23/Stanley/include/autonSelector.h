#pragma once
class button {
    
    private:
    int x;
    int y;
    int height;
    int width;
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

    };

    bool checkTouch(){ }
    void setFill(bool fill){}
};
