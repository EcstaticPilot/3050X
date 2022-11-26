#include <vex.h>
#include "sylib/sylib.h"
extern sylib::Motor F1;
extern sylib::Motor F2;
extern bool loading;
extern double TargetSpeed;
extern double GoalAngle;
extern bool VisionReady;
void pistonToggle() {
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
}

void fireDisc() {
  loading = false;
  TargetSpeed = 0; // need to create formula
  waitUntil(VisionReady &&
            (fabs(F1.get_velocity()/600 - TargetSpeed) < .25));
  pistonToggle();
  if (turretOptical.isNearObject())
    fireDisc();
  else {

    loading = true;
    TargetSpeed = 0;
  }
}

void pistonToggleReady() {
  Brain.Screen.drawRectangle(120, 190, 60, 60, orange);
  waitUntil(F1.get_velocity()/600 < TargetSpeed + .25 &&
            F1.get_velocity()/600 > TargetSpeed - .25);
  wait(10, msec);
  waitUntil(F1.get_velocity()/600 < TargetSpeed + .25 &&
            F1.get_velocity()/600 > TargetSpeed - .25);
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
}