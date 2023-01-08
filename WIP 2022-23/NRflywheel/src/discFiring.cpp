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

void fireDiscs() {
  loading = false;
  TargetSpeed = 75; // need to create formula
  waitUntil(fabs(TurretE.velocity(rpm)) < 1 &&
            fabs((FSPEED - TargetSpeed)) < 1);
  pistonToggle();
  if (turretOptical.isNearObject())
    fireDiscs();
  else {
    loading = true;
    TargetSpeed = 0;
  }
}

void pistonToggleReady() {
  Brain.Screen.drawRectangle(120, 190, 60, 60, orange);
  waitUntil(FSPEED < TargetSpeed + .25 &&
            FSPEED > TargetSpeed - .25);
  wait(10, msec);
  waitUntil(FSPEED < TargetSpeed + .25 &&
            FSPEED > TargetSpeed - .25);
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
}