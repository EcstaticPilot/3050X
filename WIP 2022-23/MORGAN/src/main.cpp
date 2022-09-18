
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       NR 7700                                                   */
/*    Created:      July, 2022                                              */
/*    Description:  code of Nikhil Ramanuja , now owned by Ashley now owned by
 * morgan                                 */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// F1                   motor         6               
// F2                   motor         5               
// Injector             digital_out   H               
// LF                   motor         10              
// LB                   motor         8               
// RF                   motor         9               
// RB                   motor         7               
// Intake1              motor         21              
// Roller               motor         20              
// Gyro                 inertial      11              
// Endgame1             digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

// A global instance of competition

#include "vex.h"
using namespace vex;
competition Competition;


int driveFactor = 1;
int GyroT = 0;
int fDesiredSpeed = 0;\
int Fspeed = F2.velocity(percent);
int B = 30;           // Flywheel target speed set with button B
int Y = 60;           // Flywheel target speed set with button Y
int X = 100;          // Flywheel target speed set with button X

float Dtraveled = 0;
float Pi = 3.14159265358979;
float D = 3.25;
float C = Pi * D;

bool intakeOn = false; // intakeOn is true when the intake is running forward
bool intakeReverse = false;
bool cancel1 = false; // used to cancel disc launch during Flywheel() function

double OldError = 0.0;
double TBHval = 0.0;
double FWDrive = 0.0;
double targetSpeed = 0.0;

// Custom Functions
void spinFlywheel(double speed) {
  speed = speed * 120; // speed is in percentage so convert to mV 100% = 12000
                       // mV
  F1.spin(forward, speed, voltageUnits::mV);
  F2.spin(forward, speed, voltageUnits::mV);
}

void driveToggle() { driveFactor = -1 * driveFactor; }
void controlFlywheelSpeed(double target) {
  double kI = .025;
  double speed = F1.velocity(percent);
  double error = target - speed;
  double fwDrive = FWDrive + kI * error;
  // :D
  Brain.Screen.printAt(1, 40, " speed = %.2f ", speed);

  // Keep drive between 0 to 100%
  if (fwDrive > 100)
    fwDrive = 100;
  if (fwDrive < 0)
    fwDrive = 0;
  // Check for zero crossing
  if (error * OldError < 0) {
    fwDrive = 0.5 * (fwDrive + TBHval);
    TBHval = fwDrive;
  }

  Brain.Screen.printAt(180, 40, "fwdrive %.1f  ", fwDrive);
  spinFlywheel(fwDrive);

  FWDrive = fwDrive;
  OldError = error;
}
void rollerStop() { Roller.stop(brake); }

void xDrive(int strafeX, int driveY, int turn, int wt) {
  // x drive code
  LF.spin(forward, driveY + strafeX + turn, percent);
  RF.spin(forward, driveY - strafeX - turn, percent);
  LB.spin(forward, driveY - strafeX + turn, percent);
  RB.spin(forward, driveY + strafeX - turn, percent);
  wait(wt, msec);
}
// void inchDrive(float D, int V){ //D is distance in inches, function needs
// testing LF.setVelocity(V, percent); RF.setVelocity(V, percent);
// RB.setVelocity(V, percent);
// LB.setVelocity(V, percent);
// RF.rotateFor(forward,(1.125)*(0.707*D)/(3.14159*3.25),rev,false);
// LF.rotateFor(forward,(1.125)*(0.707*D)/(3.14159*3.25),rev,false);
// RB.rotateFor(forward,(1.125)*(0.707*D)/(3.14159*3.25),rev,false);
// LB.rotateFor(forward,(1.125)*(0.707*D)/(3.14159*3.25),rev,true);
//}
void driveBrake() {
  LF.stop(brake);
  RF.stop(brake);
  RB.stop(brake);
  LB.stop(brake);
}
// Delete this message and the below message IF the strafing error is solved
// The side-strafing error is caused by X drive function or a hardware error,
// and is not a result of new changes
void inchDrive(float target,
               bool dir = true) { // dir=true is forward, dir=false is reverse
  target = fabs(target);
  float x = 0.0;
  LF.setPosition(0.0, rev);
  RF.setPosition(0.0, rev);
  if (dir == true) {
    while (x <= target) {
      xDrive(0, 100, 0, 10);
      x = LF.position(rev) * 1.41421 * C;
    }
  }
  if (dir == false) {
    target = -1 * target;
    while (x >= target) {
      xDrive(0, -100, 0, 10);
      x = LF.position(rev) * 1.41421 * C;
    }
  }
  driveBrake();
}
void inchStrafe(float target, bool dir) { // true:right, false:left
  target = fabs(target);
  float x = 0.0;
  LF.setPosition(0.0, rev);
  RF.setPosition(0.0, rev);
  if (dir == true) {
    while (x <= target) {
      xDrive(100, 0, 0, 10);
      x = LF.position(rev) * 1.41421 * C;
    }
  }
  if (dir == false) {
    target = -1 * target;
    while (x >= target) {
      xDrive(-100, 0, 0, 10);
      x = LF.position(rev) * 1.41421 * C;
    }
  }
  driveBrake();
}

void RunIntake() { Intake1.spin(forward, 100, percent); }

void StopIntake() { Intake1.stop(brake); }
void ReverseIntake() { // intended to unload excess discs legally
  Intake1.spin(reverse, 100, percent);
}
// this function has been tested and proven accurate
void gyroTurn(bool dir,
              float A) { // dir=true is right turn, dir=false is left turn
  int FastSpeed = 70;
  int Slowspeed = 20;
  int AngDistOfDecc = 40;
  int AngDistOfEarlyStop = 3;
  if (dir == false) {
    GyroT = -1 * A;
    float b = 0.0;
    float OldMeasure = Gyro.rotation(degrees);
    while (b - AngDistOfDecc > GyroT) {
      LF.spin(reverse, FastSpeed, percent);
      RF.spin(forward, FastSpeed, percent);
      LB.spin(reverse, FastSpeed, percent);
      RB.spin(forward, FastSpeed, percent);
      b = Gyro.rotation(degrees) - OldMeasure;
    }
    while (b - AngDistOfEarlyStop > GyroT) {
      LF.spin(reverse, Slowspeed, percent);
      RF.spin(forward, Slowspeed, percent);
      LB.spin(reverse, Slowspeed, percent);
      RB.spin(forward, Slowspeed, percent);
      b = Gyro.rotation(degrees) - OldMeasure;
    }
    driveBrake();
  }
  if (dir == true) {
    GyroT = A;
    float b = 0.0;
    float OldMeasure = Gyro.rotation(degrees);
    while (b + AngDistOfDecc < GyroT) {
      LF.spin(forward, FastSpeed, percent);
      RF.spin(reverse, FastSpeed, percent);
      LB.spin(forward, FastSpeed, percent);
      RB.spin(reverse, FastSpeed, percent);
      b = Gyro.rotation(degrees) - OldMeasure;
    }
    while (b + AngDistOfEarlyStop < GyroT) {
      LF.spin(forward, Slowspeed, percent);
      RF.spin(reverse, Slowspeed, percent);
      LB.spin(forward, Slowspeed, percent);
      RB.spin(reverse, Slowspeed, percent);
      b = Gyro.rotation(degrees) - OldMeasure;
    }
    driveBrake();
  }
}

// define your global instances of motors and other devices here

void spinRoller(float speed, float time) {
  Roller.spin(forward, speed, percent);
  wait(time, msec);
  Roller.stop(brake);
}

// Accelerates the flywheel to speed, and Flashes green when ready to launch
// d i n o s a u r
void Flywheel(int T){ //T is target speed in percent
cancel1=false;
int Fspeed=F2.velocity(percent);
while((Fspeed<T-5 or Fspeed>T+5)&&!cancel1){
F1.spin(reverse, Fspeed+(T-Fspeed), percent);
F2.spin(forward, Fspeed+(T-Fspeed), percent);
if(Controller1.ButtonRight.pressing()){
  cancel1=true;
}
}
if(cancel1==true){
Brain.Screen.drawRectangle(0,0,480,272,"red");
}
else{
Brain.Screen.drawRectangle(0,0,480,272,"green");
}
}
void launch(){ //used for launching the disc with the piston when ready
  if(Controller1.ButtonR1.pressing()){
    Injector.set(true);
    wait(0.5,sec);
    Injector.set(false); //pushes disc into the wheel to launch and resets
  Brain.Screen.clearScreen();
    }
  if(Controller1.ButtonRight.pressing()){
    F1.stop(brake); //used to enable launch cancellation
    F2.stop(brake);
    Brain.Screen.clearScreen();
  }

}// d i n o s a u r

void flywheelMonitor() {
  double current1 = F1.current();
  double current2 = F2.current();
  double t1 = F1.temperature(celsius);
  double t2 = F2.temperature(celsius);
  double b = Brain.Battery.capacity();

  Brain.Screen.printAt(1, 60, "F1 current = %.1f   Temp = %.1f   ", current1,
                       t1);
  Brain.Screen.printAt(1, 80, "F2 current = %.1f   Temp = %.1f   ", current2,
                       t2);
  Brain.Screen.printAt(1, 100, "Battery Capacity  = %.1f      ", b);
}
void pistonToggle() {

  Injector.set(true);
  wait(.5, sec);
  Injector.set(false);
}

void RollerSpinFor(float deg, bool dir) {
  float ratio = 2.375 / 3;
  if (dir == true) {
    Roller.spinFor(deg * ratio, degrees);
    Roller.stop(brake);
  }
  if (dir == false) {
    Roller.spinFor(-1 * deg, degrees);
    Roller.stop(brake);
  }
}

void intakeSpin() {
  if (intakeOn) {
    Intake1.spin(forward, 100, percent);
  } else {
    Intake1.stop(coast);
  }
}
void toggleIntake() {
  intakeOn = !intakeOn;
  intakeSpin();
}

void endgame_release() {

Endgame1.set(true);



}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void) { // needs testing
  inchStrafe(1, false);
  RollerSpinFor(90, true);
  inchStrafe(1, true);
  inchDrive(2 * 24, false);
  gyroTurn(true, 90);
  inchDrive(2 * 24 - 2);
  inchStrafe(2 * 24 - 6, true);
  inchDrive(2 * 24);
  inchStrafe(3, true);
  controlFlywheelSpeed(70);
  RollerSpinFor(90, true);
  inchStrafe(6, false);
  gyroTurn(false, 80);
  pistonToggle();
  wait(3000, msec);
  pistonToggle();
}


void usercontrol(void) {

  Controller1.ButtonL2.pressed(toggleIntake);
  Controller1.ButtonUp.pressed(endgame_release);

  while (true) {
    flywheelMonitor();

    // roller spin logic
    if (Controller1.ButtonA.pressing()) {
      spinRoller(50, 10);
    }
    if (!(Controller1.ButtonA.pressing())) {
      Roller.stop(brake);
    }
    // flywheel
    //i like umbrellas
     if(Controller1.ButtonR2.pressing()){ //refer to the variable definitions
   //to change the target speed
   if(abs(Fspeed-50)<20){ F1.spin(reverse,
   Fspeed+(50-Fspeed), percent); F2.spin(forward, Fspeed+(50-Fspeed), percent);
     }
     if(!(abs(Fspeed-50)<20)){
     F1.spin(reverse,Fspeed+(50-Fspeed)/abs(50-Fspeed)*20,percent);
     F2.spin(reverse,Fspeed+(50-Fspeed)/abs(50-Fspeed)*20,percent);
     }
     if(!(Fspeed<50-5 or Fspeed>50+5)){
       Brain.Screen.drawRectangle(0,0,480,272,"green");
     }
   }//i like umbrellas
    if (Controller1.ButtonL1.pressing()) {
      pistonToggle();
      Brain.Screen.clearScreen();
    }
    if (Controller1.ButtonR2
            .pressing()) { // refer to the variable definitions to change speed
      targetSpeed = 60;
    }
    if (Controller1.ButtonR1
            .pressing()) { // refer to the variable definitions to change speed
      targetSpeed = 75;
    }

    if (Controller1.ButtonX.pressing()) {
      F1.stop(coast);
      F2.stop(coast);
      targetSpeed = 0;
      Brain.Screen.clearScreen();
    }
    // x drive code

    if (Controller1.ButtonB.pressing()) {
      driveToggle();
    }

    int ax1 = Controller1.Axis1.position();
    int ax2 = Controller1.Axis2.position();
    int ax4 = Controller1.Axis4.position();

    xDrive(ax1, ax2, ax4, 10);
    controlFlywheelSpeed(targetSpeed);
    wait(10, msec);
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
