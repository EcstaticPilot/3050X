// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// F2                   motor         15              
// Injector             digital_out   A               
// LF                   motor         18              
// LB                   motor         12              
// RF                   motor         20              
// RB                   motor         4               
// Intake1              motor         1               
// turret               motor         9               
// gyro1                inertial      11              
// RotationL            rotation      5               
// RotationB            rotation      3               
// turretG              inertial      14              
// Color                optical       7               
// TurretE              rotation      17              
// turretOptical        optical       2               
// roller               motor         6               
// expansion            digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>

using namespace vex;

competition Competition;

// declaring external variables
bool isRed=false;

extern double GoalAngle;
extern float offset;
extern int mode;
extern bool loading;
extern double TargetSpeed;
extern double X,Y;
extern bool VisionReady;
extern bool TurretToggle;
extern double GoalAngle;
extern bool Far_Side;
extern bool Near_Side;
extern float FSPEED;

// declaring external functions
// drive.cpp
void drive(int lSpeed, int rSpeed, double wt);
void drive_brake();
void rotate(double dir, double accuracy = 1);
void inchDrive(float dist, float accuracy = 1);
void forward_dist(float dist);
void DriveToPoint(double targetX, double targetY, float speedMult = 1);
void RAMSETE(float targetX, float targetY, float targetAngle,float accuracy=1);
void DriveToPoint2(float targetX,float targetY);
//flywheel.cpp
void spinFlywheel(double speed);
int controlFlywheelSpeed();
//odometry.cpp
int odometery();
//turret.cpp
void toggleTurret();
int turretStable();
void turretSpinTo(double targetAngle, bool global);
//discFiring.cpp
void pistonToggle();
void fireDiscs();
void pistonToggleReady();
void draw_GUI();
void fetch_touch();

//////////////////////////////////////////////////////////////////////////////////

int ControllerPrint() {

  Brain.Timer.reset();
 
  while (1) {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Spd=%.2f tSpd=%.2f   ", FSPEED, TargetSpeed);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller1.Screen.setCursor(3, 1);
    switch(mode){
      case 1:Controller1.Screen.print("usingCamera.    ");
      break;
      case 2:Controller1.Screen.print("no goal.        ");
      break;
      case 3:Controller1.Screen.print("going to goal   ");
      break;
      case 4:Controller1.Screen.print("loading.        ");
      break;
    }
    this_thread::sleep_for(50);
  }
}

void pre_auton(void) {

  vexcodeInit();
  if (!(RB.installed() && LB.installed() && RF.installed() &&
        LF.installed() &&                            
        F2.installed()            
        && Intake1.installed() && turret.installed() 
        && gyro1.installed() && RotationL.installed() &&
        RotationB.installed() 
        && turretG.installed() && TurretE.installed() &&
        turretOptical.installed() && 
        Color.installed()))          
    Controller1.rumble("-------------------------------------------------------"
                       "-------------------------------");
  
  gyro1.calibrate();
  turretG.calibrate();
  waitUntil(!(gyro1.isCalibrating() && turretG.isCalibrating()));

  thread flywheelgo = thread(controlFlywheelSpeed);
  thread odometeryTracking = thread(odometery);
  thread turretStablization = thread(turretStable);
}

void autonomous(void) {

if (Far_Side) {
Brain.Screen.clearScreen();
Brain.Screen.printAt(20, 20, "Far Side Auton Running");
forward_dist(1);
rotate(90);
forward_dist(27);
rotate(180);
forward_dist(7);
roller.spin(forward, 100, pct);
// to ensure contact w/ roller
LF.spin(forward, 50, pct);
RF.spin(forward, 50, pct);
LB.spin(forward, 50, pct);
RB.spin(forward, 50, pct);
if (isRed == true) {
  waitUntil(Color.color() == blue);
}
else {
  waitUntil(Color.color() == red);
}
roller.stop();
drive_brake();
}

if (Near_Side) {
Brain.Screen.clearScreen();
Brain.Screen.printAt(20, 20, "Near Side Auton Running");
LF.spin(forward, 50, pct);
RF.spin(forward, 50, pct);
LB.spin(forward, 50, pct);
RB.spin(forward, 50, pct);
roller.spin(forward, 100, pct);
if (isRed == true) {
  waitUntil(Color.color() == blue);
}
else {
  waitUntil(Color.color() == red);
}
roller.stop();
drive_brake();
}

// loading = false;
// TargetSpeed=70;
// waitUntil(FSPEED> 68&&FSPEED<72);
// pistonToggle();
// waitUntil(FSPEED > 68&&FSPEED<72);
// pistonToggle();
// loading = true;
}

bool intakeOn = false;
void toggleIntake() { intakeOn = !intakeOn; }
bool driveDir = 0;
void driveSwitch() { driveDir = !driveDir; }

void usercontrol(void) {
  thread ControllerPrinting = thread(ControllerPrint);

  while (true) {
    if (Controller1.ButtonY.pressing()) {
      expansion.set(true);
    }
    if (Controller1.ButtonL1.pressing()) {
      TargetSpeed -= 0.5;
      wait(10, msec);
    }
    if (Controller1.ButtonR1.pressing()) {
      TargetSpeed += 0.5;
      wait(10, msec);
    }
    if(Controller1.ButtonUp.pressing()){
      TargetSpeed=75;
    }

  if(Controller1.ButtonDown.pressing())TargetSpeed=0;

  if (driveDir == false) {
    if (Controller1.ButtonL2.pressing()) {
      roller.spin(forward, 100, pct);
    }
    if (Controller1.ButtonR2.pressing()) {
      roller.spin(forward, -100, pct);
    }
  }

    if (intakeOn) {
      Intake1.spin(forward, 12, volt);
    } 
    if (!intakeOn) {
      Intake1.stop();
    }

    if ((isRed?Color.color() == red:Color.color()==blue)  && Color.isNearObject()) {
      roller.spin(forward, 200, rpm);
    }
    else {
      if(Controller1.ButtonL2.pressing())roller.spin(forward, 100, pct);
      else if(Controller1.ButtonR2.pressing())roller.spin(reverse, 100, pct);
      else roller.stop();
    }

    Color.setLightPower(100);

    if (Color.isNearObject())
      Color.setLight(ledState::on);
    else
      Color.setLight(ledState::off);
 
    if (driveDir) {
      LF.spin(forward, Controller1.Axis3.position() * 120, voltageUnits::mV);
      RF.spin(forward, Controller1.Axis2.position() * 120, voltageUnits::mV);
      LB.spin(forward, Controller1.Axis3.position() * 120, voltageUnits::mV);
      RB.spin(forward, Controller1.Axis2.position() * 120, voltageUnits::mV);
    } else if (!driveDir) {
      LF.spin(reverse, Controller1.Axis2.position() * 120, voltageUnits::mV);
      RF.spin(reverse, Controller1.Axis3.position() * 120, voltageUnits::mV);
      LB.spin(reverse, Controller1.Axis2.position() * 120, voltageUnits::mV);
      RB.spin(reverse, Controller1.Axis3.position() * 120, voltageUnits::mV);
    }
    if (Controller1.Axis2.position() == 0 &&
        Controller1.Axis3.position() == 0) {
      LF.stop(coast);
      RF.stop(coast);
      LB.stop(coast);
      RB.stop(coast);
    }
    wait(10, msec);
  }
}

int main() {

  draw_GUI();
  Brain.Screen.pressed(fetch_touch);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  Controller1.ButtonB.pressed(toggleIntake);
  Controller1.ButtonLeft.pressed(pistonToggle);
  Controller1.ButtonRight.pressed(driveSwitch);
  Controller1.ButtonX.pressed(toggleTurret);
  Controller1.ButtonA.pressed(fireDiscs);

  pre_auton();

   while (true) {
     wait(100, msec);
   }
}