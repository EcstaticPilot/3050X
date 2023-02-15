#include <vex.h>

bool Far_Side = false;
bool Near_Side = false;

void draw_GUI() {
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select side. (Tip: Far side does not include roller.)");
  Brain.Screen.setFillColor(yellow);
  Brain.Screen.drawRectangle(20, 50, 100, 100);
  Brain.Screen.printAt(25, 75, "Far");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(170, 50, 100, 100);
  Brain.Screen.printAt(175, 75, "Near");
  //Brain.Screen.setFillColor(black);
}

void fetch_touch() {
  float x_touch = Brain.Screen.xPosition();
  float y_touch = Brain.Screen.yPosition();

  if (x_touch >= 20 && x_touch <= 120 && y_touch >= 50 && y_touch <= 150) {
    Far_Side = true;
    Brain.Screen.printAt(20, 200, "Far Side Selected");
  }
  if (x_touch >= 170 && x_touch <= 270 && y_touch >= 50 && y_touch <= 150) {
    Near_Side = true;
    Brain.Screen.printAt(20, 200, "Near Side Selected");
  }
}