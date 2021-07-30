/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*    Update:  DK  6-11-2021                                                  */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LFMotor20            motor         20              
// LBMotor10            motor         10              
// RFMotor11            motor         11              
// RBMotor1             motor         1               
// Gyro                 inertial      9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global Variables here
float pi=3.14;


// Custom Functions



void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  wait(2000, msec);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting initial positions, ...
}

void drive(int lspeed, int rspeed, int wt){
  LFMotor20.spin(forward, lspeed, pct);
  LBMotor10.spin(forward, lspeed, pct);
  RBMotor1.spin(forward, rspeed, pct);
  RFMotor11.spin(forward, rspeed, pct);
  wait(wt, msec);
}

//PID TURN D = DERIVITIVE, SLOPE OF A FUNCTION
void gyroturn(float target){ //turning gyro 
 float kp=2.0;
 float kd = 16.0;
  Gyro.setRotation(0, degrees); //setting gyro to 0
  float heading = 0.0;
  float speed = 100;
  float error = target;
  float olderror=error;
  while(fabs(error)>2.0){ //fabs = absolute value
    heading= Gyro.rotation(degrees);
    olderror=error;
    error = target-heading; //the error gets smaller when u reach ur target
    drive(speed, -speed, 10);
    speed = kp*error+kd*(error-olderror); //when error is big go fast when error is small go slow, find new error
    Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", heading); // formula for 2 decimal places
    Brain.Screen.printAt(1, 60, "speed = %0.2f    degrees", speed);
  }
  drive(0,0,0);
  heading= Gyro.rotation(degrees);
  Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", heading);
}

void auton() {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}


void driver() {
  // User control code here, inside the loop
  while (true) {
    
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);

  // Run the pre-autonomous function.
  pre_auton();
  gyroturn(-90.0);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
