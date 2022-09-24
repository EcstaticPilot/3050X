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
// Gyro1                inertial      1               
// Motor                motor         2               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

void drive(int mspeed, int wt)
{
  Motor.spin(forward, mspeed, percent);
  wait(wt,msec);
}

void gyroTurn(float target)
{
float heading=Gyro1.rotation();//heading();; //initialize a variable for heading
float accuracy=2.0; //how accurate to make the turn in degrees
float error=target-heading;
float kp=5.0;
float speed=kp*error;

while(fabs(error)>=accuracy)
{
  speed=kp*error;
  drive(speed, 10); //turn right at half speed
  heading=Gyro1.rotation();//heading();  //measure the heading of the robot
  error=target-heading;  //calculate error
  Brain.Screen.printAt(1, 40, "error = %.2f degrees",error);
  Brain.Screen.printAt(1, 60, "speed = %.2f percent",speed);

}
drive(0, 0);  //stop the drive
}

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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
  Brain.Screen.printAt(1, 90, "a to reset gyro");
gyroTurn(45);
Brain.Screen.printAt(1, 100, "First turn complete");
wait(5000, msec);
gyroTurn(90);
Brain.Screen.printAt(1, 100, "Second turn complete, wait five seconds");
wait(5000, msec);

while(true) {
  if (Controller1.ButtonA.pressing()){
    Gyro1.setHeading(0.0, degrees);
  }
  wait(100, msec);
}

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
