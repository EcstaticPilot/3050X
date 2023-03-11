#include <vex.h>

extern bool loading;
extern double TargetSpeed;
extern double GoalAngle;
extern bool VisionReady;
extern double speedOffset;
extern float FSPEED;

void pistonToggle() {
  if (turretOptical.isNearObject()) {
    
    Injector.set(true);
    speedOffset+=5;
    wait(140, msec);
    Injector.set(false);
    wait(30, msec);
    speedOffset-=5;
  }
}

void fireDiscs() {
  loading = false;
// need to create formula
  waitUntil(//fabs(TurretE.velocity(rpm)) < 1 &&
            fabs((FSPEED - TargetSpeed)) < 2);
  pistonToggle(); 
  if (turretOptical.isNearObject())
    fireDiscs();
  else {
    loading = true;
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