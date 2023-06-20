/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nikhi                                                     */
/*    Created:      3/14/2023, 3:29:14 PM                                     */
/*    Description:  Stanley Controller                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;



void toggleclaw()
{
  claw.set(!claw.value());
}
int ControllerPrint()
{

  while (1)
  {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%.1f", gyro1.yaw(deg));
    this_thread::sleep_for(100);
  }
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

void pre_auton(void)
{
  Brain.Screen.drawRectangle(0, 0, 480, 240, red);
  gyro1.calibrate();
  waitUntil(gyro1.isCalibrating() == false);

  thread ControllerPrinting = thread(ControllerPrint);
  thread posTrack = thread(odometery);
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

void autonomous(void)
{
  Controller1.rumble("--");
  std::cout << "Autonomous Started" << std::endl;
  // allocate memory space using malloc
  // assign x and y values for the points
  float points[4][2] = {
      {0, 0},
      {0, 50},
      {24, 24},
      {24, 70}};
  stanley(points, sizeof(points) / 8);
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

void usercontrol(void)
{
  // User control code here, inside the loop

  while (1)
  {
    // std::cout << X << "," << Y << std::endl;
    LF.spin(forward, Controller1.Axis3.position() * 120, voltageUnits::mV);
    RF.spin(forward, Controller1.Axis2.position() * 120, voltageUnits::mV);
    LB.spin(forward, Controller1.Axis3.position() * 120, voltageUnits::mV);
    RB.spin(forward, Controller1.Axis2.position() * 120, voltageUnits::mV);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  Controller1.ButtonA.pressed(toggleclaw);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
