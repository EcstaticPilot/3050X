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
// LFDrive              motor         2               
// LBDrive              motor         1               
// LUDrive              motor         3               
// RFDrive              motor         9               
// RBDrive              motor         10              
// RUDrive              motor         8               
// FClaw                digital_out   A               
// FClaw2               digital_out   B               
// Clamp                digital_out   D               
// Clamp2               digital_out   E               
// Lift                 motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;


// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

float d = 4.0; //Global Wheel Diameter
float pi = 3.1415926535897932384626;
float g = 7/5;
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


void stopDrive(){
  LFDrive.stop();
  LBDrive.stop();
  LUDrive.stop();
  RFDrive.stop();
  RBDrive.stop();
  RUDrive.stop();
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

void inchDrive(float target, int speed){
  float c = 0; //Current Location
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
  stopDrive();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  setHold();
  inchDrive(70, 100); //Rush Yellow
  setCoast();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
   bool reversed = false;
   bool aDown = 0; //Variable for when you're trying to reverse
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

   
    
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
      if ((Controller1.ButtonR1.pressing() && !reversed) || (Controller1.ButtonR2.pressing() && reversed)){
        Lift.spin(fwd, 100, pct);
      }
      else if ((Controller1.ButtonR2.pressing() && !reversed) || (Controller1.ButtonR1.pressing() && reversed)){
        Lift.spin(reverse, 100, pct);
      }
      else{
        Lift.stop();
      }
    

    //Locking Drive
    if (Controller1.ButtonX.pressing()) {
      setHold();
    } else if (Controller1.ButtonY.pressing()) {
      setCoast();
    }


    //Front Claw
    if (Controller1.ButtonR1.pressing()){
      FClaw.set(true);
      FClaw2.set(true);
    }
    else if (Controller1.ButtonR2.pressing()){
      FClaw.set(false);
      FClaw2.set(false);
    }

    

    //Clamp
    if (Controller1.ButtonUp.pressing()){
      Clamp.set(true);
      Clamp2.set(true);
    }
    else if (Controller1.ButtonDown.pressing()){
      Clamp.set(false);
      Clamp2.set(false);
    }

    



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
