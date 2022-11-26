
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       NR 7700P                                                  */
/*    Created:      May 13, 2022                                              */
/*    Description:  code of Nikhil Ramanuja 7700P turret bot vex spin up      */
/*----------------------------------------------------------------------------*/
// vex::vision::signature BGOAL = vex::vision::signature (1, -1855, 2553, 349,
// 2243, 10039, 6141, 1, 0);
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// F1                   motor         2
// F2                   motor         15
// Injector             digital_out   A
// LF                   motor         18
// LB                   motor         12
// RF                   motor         20
// RB                   motor         4
// Intake1              motor         1
// turret               motor         21
// gyro1                inertial      11
// RotationL            rotation      5
// RotationB            rotation      3
// turretG              inertial      14
// Color                optical       7
// TurretE              rotation      17
// turretOptical        optical       9
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "stdio.h"
#include "sylib/sylib.h"
#include "vex.h"
#include <math.h>

// including files

using namespace vex;

// A global instance of competition

competition Competition;
extern sylib::Motor F1;
extern sylib::Motor F2;
// declaring external variables
extern double GoalAngle;
extern float offset;
extern bool loading;
extern double TargetSpeed;
extern double X, Y;
extern bool VisionReady;
extern bool TurretToggle;
// team switch
bool IsRed = true;

/*

PROTOTYPES FOR FUNCTIONS

*/
// drive.cpp
void drive(int lSpeed, int rSpeed, double wt);
void drive_brake();
void rotate(double dir, double accuracy = 1);
void inchDrive(float dist, float accuracy = 1);
void driveToPoint(float targetX,float targetY,float endOrientation = gyro1.rotation(degrees));
// flywheel.cpp
void spinFlywheel(double speed);
int controlFlywheelSpeed();
// odometry.cpp
int odometery();
// turret.cpp
void toggleTurret();
int turretStable();
void turretSpinTo(double targetAngle, bool global);
// discFiring.cpp
void pistonToggle();
void fireDisc();
void pistonToggleReady();

/*

CONTROLLER PRINTING

*/
int ControllerPrint() {

  Brain.Timer.reset();

  while (1) {
    Controller2.Screen.setCursor(1, 1);
    double speed = F1.get_velocity() / 600;
    Controller2.Screen.print("Spd=%.2f tSpd=%.2f   ", speed, TargetSpeed);
    Controller2.Screen.setCursor(2, 1);
    Controller2.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller2.Screen.setCursor(3, 1);
    Controller2.Screen.print("time=%.2f ", Brain.timer(sec));

    if (Brain.timer(sec) == 15) {
      Controller1.rumble(".");
    } // 2 minute mark
    if (Brain.timer(sec) == 45) {
      Controller1.rumble("..");
    } // 1:30 mark
    if (Brain.timer(sec) == 75)
      Controller1.rumble("...");
    // 1 minute mark
    if (Brain.timer(sec) == 105) {
      Controller1.rumble("....");
    } // 30 second mark

    this_thread::sleep_for(50);
  }
}

/*---------------------------------------------------------------------------*/
/*                          Pre-autonomousomous Functions                    */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  
  vexcodeInit();
  sylib::initialize();
  if (!(RB.installed() && LB.installed() && RF.installed() &&
        LF.installed() // &&                            // drive motors
        //  F1.installed() && F2.installed()             // flywheel
        && Intake1.installed() && turret.installed() // turret and intake
        && gyro1.installed() && RotationL.installed() &&
        RotationB.installed() // odom stuff
        && turretG.installed() && TurretE.installed() &&
        turretOptical.installed() && // turret sensors
        Color.installed()))          // roler sensor
    Controller1.rumble("-------------------------------------------------------"
                       "-------------------------------");

  // Initializing Robot Configuration. DO NOT REMOVE!

  gyro1.calibrate();
  turretG.calibrate();
  waitUntil(!gyro1.isCalibrating() && !turretG.isCalibrating());
  // launch threads
  thread flywheelgo = thread(controlFlywheelSpeed);
  thread odometeryTracking = thread(odometery);
  thread turretStablization = thread(turretStable);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here .  */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  // thread turretStablization = thread(turretStable);
  /*

    inchDrive(0.3);
    rotate(90);
    inchDrive(24);
    rotate(180);
    inchDrive(0.1);
    Intake1.spin(forward, 100, pct);
    waitUntil(Color.color() == blue);
    Intake1.stop();&*/

  inchDrive(0.3);
  rotate(-90);
  inchDrive(24);
  rotate(-90);
  inchDrive(0.1);
  Intake1.spin(forward, 100, pct);
  waitUntil(Color.color() == red);
  Intake1.stop();
  
  turretSpinTo(atan2(X - 110, 110 - Y) * (180 / M_PI), true);
  GoalAngle = atan2(X - 115, 115 - Y) * (180 / M_PI);
  loading = false;
  spinFlywheel(100);
  loading = false;
  waitUntil(F1.get_velocity() / 600 > 99 && F1.get_velocity() / 600 < 101);
  pistonToggle();
  loading = false;
  waitUntil(F1.get_velocity() / 600 > 99 && F1.get_velocity() / 600 < 101);
  pistonToggle();
  loading = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of
 */
/*  a VEX Competition.                            */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

bool intakeOn = false;
void toggleIntake() { intakeOn = !intakeOn; }
bool driveDir = 0;
void driveSwitch() { driveDir = !driveDir; }

void usercontrol(void) {
  thread ControllerPrinting = thread(ControllerPrint);
  while (true) {

    /*

    CONTROLLER 2 SPEED CONTROL

    */
    if (Controller2.ButtonL1.pressing()) {
      TargetSpeed -= 0.5;
      wait(10, msec);
    }
    if (Controller2.ButtonR1.pressing()) {
      TargetSpeed += 0.5;
      wait(10, msec);
    }
    if (Controller2.ButtonL2.pressing()) {
      offset += 0.5;
      wait(10, msec);
    }
    if (Controller2.ButtonR2.pressing()) {
      offset -= 0.5;
      wait(10, msec);
    }

    if (Controller2.ButtonX.pressing()) {
      TargetSpeed = 0;
    }
    if (Controller2.ButtonA.pressing()) {
      TargetSpeed = 75;
    }
    if (Controller2.ButtonB.pressing()) {
      TargetSpeed = 85;
    }
    if (Controller2.ButtonY.pressing()) {
      TargetSpeed = 100;
    }
    /*

    INTAKE

    */
    if (intakeOn) {
      Intake1.spin(forward, 150, rpm);
    } else {

      if ((IsRed ? Color.color() == red : Color.color() == blue) &&
          Color.isNearObject())
        Intake1.spin(forward, 200, rpm);
      else
        Intake1.stop();
    }
    Color.setLightPower(100);
    if (Color.isNearObject())
      Color.setLight(ledState::on);
    else
      Color.setLight(ledState::off);
    /*

    TANK DRIVE CODE

    */

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

// Main will set up the competition functions and callbacks.
//
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  Controller1.ButtonB.pressed(toggleIntake);
  Controller1.ButtonLeft.pressed(pistonToggleReady);
  Controller1.ButtonRight.pressed(driveSwitch);
  Controller1.ButtonX.pressed(toggleTurret);
  Controller2.ButtonUp.pressed(pistonToggle);
  Controller2.ButtonLeft.pressed(pistonToggle);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
