#include "vex.h"
#include "autonSelector.h"
#include "sylib/sylib.hpp"
using namespace vex;
#include <string>

teamC team = tred;

//default
Auton selectedAuton = topSide;

button red1;
button red2;

button blue1;
button blue2;


button skillsButton;
button resetButton;
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
button::button(int x, int y, int height, int width, vex::color boxColor, std::string text) : x(x), y(y), height(height), width(width), boxColor(boxColor), text(text)
{
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(boxColor);
    Brain.Screen.drawRectangle(x, y, width, height);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.printAt(x, y + height / 2, text.c_str());
    Brain.Screen.setFillColor(black);
}
/**
 * @brief checks if the button is being touched and sets the fill to the return value
 *
 * @return true
 * @return false
 */
bool button::checkTouch()
{
    int xTouch = Brain.Screen.xPosition();
    int yTouch = Brain.Screen.yPosition();
    if ((xTouch <= x + width and xTouch > x) and (yTouch <= y + height and yTouch > y))
    {
        setFill(true);
        return true;
    }
    else
    {
        setFill(false);
        return false;
    }
}
/**
 * @brief sets the button to be filled or not
 *
 * @param fill
 */
void button::setFill(bool fill)
{
    Brain.Screen.setPenColor(boxColor);
    if (fill)
    {
        Brain.Screen.drawRectangle(x, y, width, height, boxColor);
    }
    else
    {
        Brain.Screen.drawRectangle(x, y, width, height, black);
        // Brain.Screen.setPenColor(boxColor);
    }
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.printAt(x, y + height / 2, text.c_str());
    Brain.Screen.setFillColor(black);
}

void drawField(int xOffset = 0, int yOffset = 0)
{
    const color grey = color(132, 132, 132);
    const color highlighter = color(212, 254, 0);

    Brain.Screen.setPenColor(grey);
    Brain.Screen.setFillColor(grey);
    // main tile
    Brain.Screen.drawRectangle(xOffset, yOffset, 200, 200, grey);

    // auton lines
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawLine(98 + xOffset, 0 + yOffset, 98 + xOffset, 200 + yOffset);
    Brain.Screen.drawLine(102 + xOffset, 0 + yOffset, 102 + xOffset, 200 + yOffset);
    // corners
    Brain.Screen.drawLine(20 + xOffset, 0 + yOffset, 0 + xOffset, 20 + yOffset);
    Brain.Screen.drawLine(180 + xOffset, 0 + yOffset, 200 + xOffset, 20 + yOffset);
    Brain.Screen.drawLine(0 + xOffset, 180 + yOffset, 20 + xOffset, 200 + yOffset);
    Brain.Screen.drawLine(180 + xOffset, 200 + yOffset, 200 + xOffset, 180 + yOffset);

    // ladder
    Brain.Screen.setPenColor(highlighter);
    Brain.Screen.drawLine(100 + xOffset, 67 + yOffset, 133 + xOffset, 100 + yOffset);
    Brain.Screen.drawLine(133 + xOffset, 100 + yOffset, 100 + xOffset, 133 + yOffset);
    Brain.Screen.drawLine(100 + xOffset, 133 + yOffset, 67 + xOffset, 100 + yOffset);
    Brain.Screen.drawLine(67 + xOffset, 100 + yOffset, 100 + xOffset, 67 + yOffset);

    // goals
    Brain.Screen.setFillColor(highlighter);
    Brain.Screen.drawCircle(67 + xOffset, 67 + yOffset, 10);
    Brain.Screen.drawCircle(133 + xOffset, 67 + yOffset, 10);
    Brain.Screen.drawCircle(133 + xOffset, 133 + yOffset, 10);
    Brain.Screen.drawCircle(100 + xOffset, 169 + yOffset, 10);
    Brain.Screen.drawCircle(67 + xOffset, 133 + yOffset, 10);

    // red rings
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawCircle(105 + xOffset, 21 + yOffset, 5);
    Brain.Screen.drawCircle(105 + xOffset, 31 + yOffset, 5);
    Brain.Screen.drawCircle(133 + xOffset, 27 + yOffset, 5);
    Brain.Screen.drawCircle(195 + xOffset, 5 + yOffset, 5);
    Brain.Screen.drawCircle(178 + xOffset, 100 + yOffset, 5);
    Brain.Screen.drawCircle(166 + xOffset, 169 + yOffset, 5);
    Brain.Screen.drawCircle(133 + xOffset, 169 + yOffset, 5);
    Brain.Screen.drawCircle(195 + xOffset, 195 + yOffset, 5);
    Brain.Screen.drawRectangle(0 + xOffset, 85 + yOffset, 8, 30);

    // blue rings
    Brain.Screen.setFillColor(blue);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawCircle(200 - 105 + xOffset, 21 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 105 + xOffset, 31 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 133 + xOffset, 27 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 195 + xOffset, 5 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 178 + xOffset, 100 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 166 + xOffset, 169 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 133 + xOffset, 169 + yOffset, 5);
    Brain.Screen.drawCircle(200 - 195 + xOffset, 195 + yOffset, 5);
    Brain.Screen.drawRectangle(192 + xOffset, 85 + yOffset, 8, 30);
    Brain.Screen.setFillColor(black);
}

void onScreenPress()
{
    std::cout << "screen touch";
    if (red1.checkTouch()){
        selectedAuton = topSide;
        team = tred;
    }
    if (red2.checkTouch()){
        selectedAuton = bottomSide;
        team = tred;
    }
    if (blue1.checkTouch()){
        selectedAuton = topSide;
        team = tblue;
    }
    if (blue2.checkTouch()){
        selectedAuton = bottomSide;
        team = tblue;
    }
    if (skillsButton.checkTouch()){
        selectedAuton = skills;
        team = tred;
    }
    if (resetButton.checkTouch())
    {
        gyro1.calibrate();
        waitUntil(!gyro1.isCalibrating());
        robot = point(0, 0);
        resetButton.setFill(false);
    }
    // Brain.Screen.print(selectedAuton);
    std::cout << selectedAuton;
}

void initializeAutonSelector(void)
{
    drawField(75, 25);

    red2 = button(75, 160, 40, 40, red, " 2");
    red1 = button(70, 50, 40, 40, red, " 1");
    blue1 = button(235, 50, 40, 40, blue, " 1");
    blue2 = button(235, 160, 40, 40, blue, " 2");

    skillsButton = button(345, 60, 110, 60, yellow, "skills");

    resetButton = button(425, 60, 110, 60, red, "reset");
}