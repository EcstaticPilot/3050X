
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
// Controller2          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
//including files

using namespace vex;


// A global instance of competition

competition Competition;

//declaring external variables
bool isRed=false;
extern double GoalAngle;
extern float offset;
extern bool loading;
extern double TargetSpeed;
extern double X,Y;
extern bool VisionReady;
extern bool TurretToggle;
/*

FUNCTIONS

*/
//drive.cpp
void drive(int lSpeed, int rSpeed, double wt);
void drive_brake();
void rotate(double dir, double accuracy = 1);
void inchDrive(float dist, float accuracy = 1);
void DriveToPoint(double targetX, double targetY, float speedMult = 1);
void RAMSETE(float targetX, float targetY, float targetAngle,float accuracy=1);
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
void fireDisc();
void pistonToggleReady();

/*

CONTROLLER PRINTING

*/
extern float FSPEED;
int ControllerPrint() {

  Brain.Timer.reset();
 
  while (1) {
   
   
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Spd=%.2f tSpd=%.2f   ", FSPEED, TargetSpeed);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("distance=%.2f ",sqrt( (115-X)*(115-X)+(115-Y)*(115-Y)));
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
  if (!(RB.installed() && LB.installed() && RF.installed() &&
        LF.installed() &&                            // drive motors
        F1.installed() && F2.installed()             // flywheel
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
  waitUntil(!(gyro1.isCalibrating() && turretG.isCalibrating()));
  //launch threads
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

  /*
  // thread turretStablization = thread(turretStable);

  inchDrive(0.3);
  rotate(-90);
  inchDrive(12);
  rotate(0);
  inchDrive(4.0);
  Intake1.spin(forward, 100, pct);
  Color.setLightPower(50);
  waitUntil(Color.color() == blue);
  wait(30, msec);
  Intake1.stop();

  /*
   inchDrive(0.3);
   rotate(-90);
   inchDrive(24);
   rotate(-90);
   inchDrive(0.1);
   Intake1.spin(forward, 100, pct);
   waitUntil(Color.color() == red);
   Intake1.stop();
   
*/

/*
loading = false;
  TargetSpeed=100;
  waitUntil(FSPEED> 99&&FSPEED<101);
  pistonToggle();
  waitUntil(FSPEED > 99&&FSPEED<101);
  pistonToggle();
  loading = true;*/
  DriveToPoint(50, -25,1);
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
  X = 75;
  Y = 0;
    //thread turretStablization = thread(turretStable);

 // thread flywheelgo = thread(controlFlywheelSpeed);
  thread ControllerPrinting = thread(ControllerPrint);
  while (true) {

    /*

    CONTROLLER 2 SPEED CONTROL

    */
    if (Controller1.ButtonL1.pressing()) {
      TargetSpeed -= 0.5;
      wait(10, msec);
    }
    if (Controller1.ButtonR1.pressing()) {
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
    if(Controller1.ButtonUp.pressing()){
      TargetSpeed=75;
    }

  if(Controller1.ButtonDown.pressing())TargetSpeed=0;
    /*

    INTAKE

    */
    if (intakeOn) {
      Intake1.spin(forward, 80, pct);
    } else {

      if ((isRed?Color.color() == red:Color.color()==blue)  && Color.isNearObject())
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
  Controller1.ButtonLeft.pressed(pistonToggle);
  Controller1.ButtonRight.pressed(driveSwitch);
  Controller1.ButtonX.pressed(toggleTurret);
  Controller1.ButtonA.pressed(fireDisc);
  Controller2.ButtonUp.pressed(pistonToggleReady);
  Controller2.ButtonLeft.pressed(pistonToggle);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
