#include "vex.h"
#include "autonSelector.h"
#include "sylib/sylib.hpp"
using namespace vex;
#include <string>

Auton selectedAuton = skills;

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
    Brain.Screen.setPenColor(boxColor);
    Brain.Screen.drawRectangle(x, y, width, height);
    Brain.Screen.printAt(x, y + height / 2, text.c_str());
}
/**
 * @brief checks if the button is being touched
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

    Brain.Screen.printAt(x, y + height / 2, text.c_str());
}

void drawField()
{

    color grey = color(132, 132, 132);

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

    // V2
    int yOffset = -20;
    int xOffset = 0;
    Brain.Screen.setFillColor(grey);
    Brain.Screen.setPenColor(grey);
    Brain.Screen.drawRectangle(50 + xOffset, 15 + yOffset, 250, 250);
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawRectangle(132 + xOffset, 54 + yOffset, 87, 5);
    Brain.Screen.drawRectangle(173 + xOffset, 55 + yOffset, 5, 172);
    Brain.Screen.drawRectangle(132 + xOffset, 222 + yOffset, 86, 5);
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawRectangle(255 + xOffset, 95 + yOffset, 45, 90);
    Brain.Screen.drawRectangle(173 + xOffset, 227 + yOffset, 5, 38);
    Brain.Screen.setFillColor(blue);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawRectangle(50 + xOffset, 95 + yOffset, 45, 90);
    Brain.Screen.drawRectangle(173 + xOffset, 15 + yOffset, 4, 40);
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.drawCircle(173 + 2 + xOffset, 53 + 2 + yOffset, 2);
    Brain.Screen.drawCircle(173 + 2 + xOffset, 222 + 2 + yOffset, 2);
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(black);
}

void onScreenPress()
{
    std::cout << "screen touch";
    if (red1.checkTouch())
        selectedAuton = defensiveZone;
    if (red2.checkTouch())
        selectedAuton = offensiveZone;
    if (blue1.checkTouch())
        selectedAuton = offensiveZone;
    if (blue2.checkTouch())
        selectedAuton = defensiveZone;
    if (skillsButton.checkTouch())
        selectedAuton = skills;
    if (resetButton.checkTouch())
    {
        gyro1.calibrate();
        waitUntil(!gyro1.isCalibrating());
        robot = point(0,0);
        resetButton.setFill(false);
    }
    //Brain.Screen.print(selectedAuton);
    std::cout << selectedAuton;
}

void initializeAutonSelector(void)
{
    drawField();

    red1 = button(70, 190, 50, 50, red, "red1");
    red2 = button(228, 190, 50, 50, red, "red2");
    blue1 = button(70, 0, 50, 50, blue, "blue1");
    blue2 = button(228, 0, 50, 50, blue, "blue2");

    skillsButton = button(345, 60, 110, 60, yellow, "skills");

    resetButton = button(425, 60, 110, 60, red, "reset");
}