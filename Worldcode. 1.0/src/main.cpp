/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\chiep                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightBackMotor       motor         5
// LeftBackMotor        motor         6
// Arm                  motor         12
// Controller1          controller
// Lintake              motor         16
// Rintake              motor         18
// Lfront               motor         1
// Rfront               motor         10
// Inertial4            inertial      4
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
// Global Variable
float dia = 4.0;

// tis is GUI : Graphic User Interface
void draw() {

  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(10, 10, 100, 50);
  Brain.Screen.printAt(10, 10, "hi");
  Brain.Screen.printAt(10, 20, "RMotor Temp%f ",
                       LeftBackMotor.temperature(pct));
  Brain.Screen.printAt(10, 35, "LMotor Temp%f ",
                       RightBackMotor.temperature(pct));
  // Brain.Screen.printAt(10, 40,"Motor");

  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(400, 10, 80, 50);
  Brain.Screen.printAt(410, 15, "C++");
}

void motorCurrent() {
  Brain.Screen.printAt(10, 80, "LMotor current%f  ", LeftBackMotor.current());
  Brain.Screen.printAt(10, 96, "RMotor current%f  ", RightBackMotor.current());
  Brain.Screen.printAt(10, 110, "LMotor current%f ", Lfront.current());
  Brain.Screen.printAt(10, 122, "RMotor current%f ", Rfront.current());
  Brain.Screen.printAt(10, 132, "Motor current%f  ", Arm.current());
}

void InchDrive(float target, int speed) {

  int c = 0; // our distance
   LeftBackMotor.resetRotation();
  while (abs(c) <= target) {
    LeftBackMotor.spin(forward, speed, pct);
    RightBackMotor.spin(forward, speed, pct);
    Lfront.spin(forward, speed, pct);
    Rfront.spin(forward, speed, pct);

    c = LeftBackMotor.rotation(rev) * 3.14 * dia;
    
  }
  LeftBackMotor.stop();
  RightBackMotor.stop();
  Lfront.stop();
  Rfront.stop();
}

void Drive(float speed) {
  LeftBackMotor.spin(forward, speed, pct);
  RightBackMotor.spin(forward, speed, pct);
  Lfront.spin(forward, speed, pct);
  Rfront.spin(forward, speed, pct);
}

void Pdrive(float target, float distrav) {
  float error = 0;
  float Dist = 0;
  float kP = 1;
  while (true) {
    Dist = Lfront.position(degrees), (4 * 3.14 / 360);
    error = target - Dist;
    Drive(error * kP);
    wait(5, msec);
  }
}

void autonDriver(int wt, int Lspeed, int Rspeed, int Armspeed, int Lint,int Rint) 
{
  LeftBackMotor.spin(forward, Lspeed, pct);
  RightBackMotor.spin(forward, Rspeed, pct);
  Lfront.spin(forward, Lspeed, pct);
  Rfront.spin(forward, Rspeed, pct);
  Arm.spin(forward, Armspeed, pct);
  Lintake.spin(forward, Lint, pct);
  Rintake.spin(forward, Rint, pct);

  wait(wt, msec);
}

void gyroTurn(float target, int Lspeed, int Rspeed)
{
  /*  while(Inertial4.isCalibrating())
  {
    // Wait for Gyro Calibration , Sleep but Allow other tasks to run
      this_thread::sleep_for(20);
  }*/
  float heading4 = 0;
  Inertial4.setRotation(0, degrees);

  while (fabs(heading4) <= target) {
    heading4 = Inertial4.rotation(degrees);
    autonDriver(10, Lspeed, Rspeed, 0, 0, 0);
    wait(10, msec);
  }
  autonDriver(0, 0, 0, 0, 0, 0);
}

void encPrint() {

  int enc1 = Rfront.velocity(pct);
  int enc2 = Lfront.velocity(pct);
  Brain.Screen.printAt(20, 20, "Right motor front %d ", enc1);
  Brain.Screen.printAt(20, 20, "Left motor front %d ", enc2);
}

//----------------------------------------EOF-------------------------------------------------

void pre_auton(void) 
{

}

void autonomous(void) 
{
  encPrint();
  InchDrive(3, 100);
  InchDrive(3, -100);
  autonDriver(300, 50, -50, 0, -80, -80);
  autonDriver(50,0,0,0,0,0);
  InchDrive(26, 100);
  autonDriver(20,-50,-50,0,0,0);
  autonDriver(500, -50, 50, 0, -80, -80);
  autonDriver(50,0,0,0,0,0);
  autonDriver(700,0,0,80,-20,-20);
  InchDrive(5, 80);
  autonDriver(600,0,0,20,80,80);
  autonDriver(300,-80,-80,20,0,0);
  autonDriver(300,80,-80,20,0,0);
  autonDriver(50,0,0,0,0,0);
}

void driverControl() {

  while (true) {
    motorCurrent();
    Brain.Screen.printAt(160, 60, "hi");
    Brain.Screen.printAt(20, 20, "Motor Temp%f ", LeftBackMotor.temperature(pct));
    Brain.Screen.printAt(20, 40, "Motor Temp%f ", RightBackMotor.temperature(pct));
    Brain.Screen.printAt(20, 60, "Motor Temp%f ", Lfront.temperature(pct));
    Brain.Screen.printAt(20, 80, "Motor Temp%f ", Rfront.temperature(pct));

    LeftBackMotor.spin(forward, Controller1.Axis3.position(pct), pct);
    RightBackMotor.spin(forward, Controller1.Axis2.position(pct), pct);
    Lfront.spin(forward, Controller1.Axis3.position(pct), pct);
    Rfront.spin(forward, Controller1.Axis2.position(pct), pct);

    if (Controller1.ButtonR1.pressing()) {
      Lintake.spin(forward, 100, pct);
      Rintake.spin(forward, 100, pct);

    } else if (Controller1.ButtonR2.pressing()) {
      Lintake.spin(reverse, 100, pct);
      Rintake.spin(reverse, 100, pct);

    }

    else {
      Lintake.stop(brake);
      Rintake.stop(brake);
    }
    if (Controller1.ButtonL1.pressing()) {
      Arm.spin(forward, 85, pct);

    } else if (Controller1.ButtonL2.pressing()) {
      Arm.spin(reverse, 85, pct);
    }

    else {
      Arm.stop(brake);
    }
  }
}

int main() {
  vexcodeInit();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(driverControl);

  pre_auton();
  // autonomous ();
  // driverControl();
  while (true) {
    wait(100, msec);
  }
  // Run the pre-autonomous function.
}
// Initializing Robot Configuration. DO NOT REMOVE!
