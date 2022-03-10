/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LFDrive              motor         9               
// LBDrive              motor         8               
// LUDrive              motor         10              
// RFDrive              motor         3               
// RBDrive              motor         1               
// RUDrive              motor         2               
// Claw                 digital_out   A               
// Tilter               digital_out   B               
// Lift                 motor         7               
// Gyro                 inertial      6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;


// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

float d = 4.0; //Global Wheel Diameter
float pi = 3.1415926535897932384626;
float g = 7/5;
//true open
//false close

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Brain.Screen.printAt(20, 20, "LF Temp %f ", LFDrive.temperature(pct));
  Brain.Screen.printAt(20, 40, "LB Temp %f ", LBDrive.temperature(pct));
  Brain.Screen.printAt(20, 60, "LU Temp %f ", LUDrive.temperature(pct));
  Brain.Screen.printAt(20, 80, "RF Temp %f ", RFDrive.temperature(pct));
  Brain.Screen.printAt(20, 100, "RB Temp % f", RBDrive.temperature(pct));
  Brain.Screen.printAt(20, 120, "RU Temp % f", RUDrive.temperature(pct));
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void brakeDrive(){
  LFDrive.stop(brake);
  LBDrive.stop(brake);
  LUDrive.stop(brake);
  RFDrive.stop(brake);
  RBDrive.stop(brake);
  RUDrive.stop(brake);
}

void setHold(){
  LFDrive.setBrake(hold);
  LBDrive.setBrake(hold);
  LUDrive.setBrake(hold);
  RFDrive.setBrake(hold);
  RBDrive.setBrake(hold);
  RUDrive.setBrake(hold);
}

void setCoast(){
  LFDrive.setBrake(coast);
  LBDrive.setBrake(coast);
  LUDrive.setBrake(coast);
  RFDrive.setBrake(coast);
  RBDrive.setBrake(coast);
  RUDrive.setBrake(coast);
}

void inchDrive(float target, int speed, bool claw){
  float c = 0; //Current Location
  Claw.set(claw);
  LBDrive.setRotation(0, degrees);
  while (fabs(c) <= target) {
    LFDrive.spin(forward, speed, pct);
    LBDrive.spin(forward, speed, pct);
    LUDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
    RBDrive.spin(forward, speed, pct);
    RUDrive.spin(forward, speed, pct);
    c = LBDrive.rotation(rev) * pi * d * g;
  }
  brakeDrive();
}

void autonDriver(int wt, int lspeed, int rspeed, bool claw) //int liftspeed) //bool claw, //int Clawspin, int moggs) 
{

  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct); 
  Claw.set(claw);
  /*//LLift.spin(forward, liftspeed, pct);
  //RLift.spin(forward, liftspeed, pct);
  ClawSpin.spin(forward, Clawspin, pct);
  mogolift.spin(forward, moggs, pct);
  wait(wt, msec);*/
}

void gyroTurn(float target) {
  while (Gyro.isCalibrating()) {
    // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    //90 = right, -90 = left
    this_thread::sleep_for(20);
  }
  float heading69 = 0;
  Gyro.setRotation(0, degrees);

  float speed = 0.0;
  float kp = 1.0;
  float d = 2.0;

  Brain.Screen.clearScreen();
  while (fabs(target - heading69) >= d) {
    if (target - heading69 > 0) {
      speed = kp * (target - heading69) + 10;
    }
    if (target - heading69 < 0) {
      speed = kp * (target - heading69) - 10;
    }
    autonDriver(10, speed, -speed, false);;
    //Drive(10, speed, -speed);
    heading69 = Gyro.rotation(degrees);
    Brain.Screen.printAt(1, 40, "heading = %.3f", heading69);
  }
  brakeDrive();
  //Brain.Screen.clearScreen();
}


void autonomous(void) { 
  //YELLOW RUSH

  //true open
  //false close
  Tilter.set(true);
  Claw.set(true);
  wait(200, msec);
  inchDrive(90, 75, true);
  wait(200, msec);
  Claw.set(false);
  wait(500, msec);
  inchDrive(90, -75, false);
  wait(15000, msec);
  

}
void usercontrol(void) {
  // User control code here, inside the loop
   bool reversed = false;
   bool locked = false;
   bool aDown = 0; //Variable for when you're trying to reverse
   bool xDown = 0; //Variable for when you're locking the drive
   Lift.setBrake(hold);
  while (1) {
    //Reversing Drive
    if(Controller1.ButtonA.pressing() && !aDown){
      reversed = !reversed;
      aDown = true;
    }
    else if (!Controller1.ButtonA.pressing()) {
      aDown = false;
    }
    if(reversed){
      Brain.Screen.printAt(20, 140, "reversed");
    }
    else if(!reversed){
      Brain.Screen.printAt(20, 140, "normal");
    }
    

    //Locking Drive
    if(Controller1.ButtonX.pressing() && !xDown){
      locked = !locked;
      xDown = true;
    }
    else if (!Controller1.ButtonX.pressing()) {
      xDown = false;
    }
    if(locked){
      Brain.Screen.printAt(20, 160, "locked");
      setHold();
    }
    else if(!locked){
      Brain.Screen.printAt(20, 160, "coast");
      setCoast();
    }

    //Drive Code
    if(!reversed){
      LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      LUDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);
      RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
      RUDrive.spin(forward, Controller1.Axis2.position(pct), pct);
    }
    else if(reversed){
      LBDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      LFDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      LUDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      RBDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
      RFDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
      RUDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
    }
      
      //Lift Code
    if(Controller1.ButtonL1.pressing()){
      Lift.spin(fwd, 100, pct);
    }
    else if(Controller1.ButtonL2.pressing()){
      Lift.spin(reverse, 100, pct);
    }
    else{
      Lift.stop();
    }

    //Front Claw
    if (Controller1.ButtonR1.pressing()){
      Claw.set(true);
    }
    else if (Controller1.ButtonR2.pressing()){
      Claw.set(false);
    }

    //Tilter
    if (Controller1.ButtonB.pressing()){
      Tilter.set(true);
    }
    else if (Controller1.ButtonY.pressing()){
      Tilter.set(false);
    }


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


// Main will set up the competition functions and callbacks.
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
