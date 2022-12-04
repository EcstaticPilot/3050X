#include <vex.h>
extern bool loading;
extern double TargetSpeed;
extern double GoalAngle;
extern bool VisionReady;
extern float FSPEED;
void pistonToggle() {
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
}

void fireDisc() {
  loading = false;
  TargetSpeed = 75; // need to create formula
  waitUntil(Controller1.ButtonLeft.pressing() &&
            fabs((FSPEED - TargetSpeed)) < 3);
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
  waitUntil(F1.velocity(pct) < TargetSpeed + .25 &&
            F1.velocity(pct) > TargetSpeed - .25);
  wait(10, msec);
  waitUntil(F1.velocity(pct) < TargetSpeed + .25 &&
            F1.velocity(pct) > TargetSpeed - .25);
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
}