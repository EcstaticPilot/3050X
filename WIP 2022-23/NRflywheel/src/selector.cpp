#include <vex.h>

bool Far_Side = false;
bool Near_Side = false;

extern bool isRed;

void draw_GUI() {
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select side");
  Brain.Screen.setFillColor(yellow);
  Brain.Screen.drawRectangle(20, 50, 50, 50);
  Brain.Screen.printAt(25, 55, "Far");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(100, 50, 50, 50);
  Brain.Screen.printAt(105, 55, "Near");
  Brain.Screen.printAt(20, 115, "Select alliance color");
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(20, 130, 50, 50);
  Brain.Screen.printAt(25, 135, "Blue");
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(100, 130, 50, 50);
  Brain.Screen.printAt(105, 135, "Red");

}

void fetch_touch() {
  float x_touch = Brain.Screen.xPosition();
  float y_touch = Brain.Screen.yPosition();

  if (x_touch >= 20 && x_touch <= 70 && y_touch >= 50 && y_touch <= 100) {
    Far_Side = true;
    Near_Side = false;
    Brain.Screen.printAt(20, 220, "                 ");
    Brain.Screen.printAt(20, 200, "Far Side Selected");
  }
  if (x_touch >= 100 && x_touch <= 150 && y_touch >= 50 && y_touch <= 100) {
    Near_Side = true;
    Far_Side = false;
    Brain.Screen.printAt(20, 220, "                  ");
    Brain.Screen.printAt(20, 220, "Near Side Selected");
  }
  if (x_touch >= 20 && x_touch <= 70 && y_touch >= 130 && y_touch <= 180) {
    isRed = false;
    Brain.Screen.printAt(20, 240, "                      ");
    Brain.Screen.printAt(20, 240, "Blue Alliance Selected");
  }
  if (x_touch >= 100 && x_touch <= 150 && y_touch >= 130 && y_touch <= 180) {
    isRed = true;
    Brain.Screen.printAt(20, 240, "                     ");
    Brain.Screen.printAt(20, 240, "Red Alliance Selected");
  }
}