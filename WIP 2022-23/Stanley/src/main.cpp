/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Nikhil Ramanuja                                           */
/*    Created:      3/14/2023, 3:29:14 PM                                     */
/*    Description:  testing program for path following, stanley and pure pursuit*/
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

bool devicesCheck()
{
  if (LF.installed() && RF.installed() && LB.installed() && RB.installed() &&
      gyro1.installed() &&
      RotationR.installed() && RotationB.installed() && RotationL.installed())
  {
    return true;
  }
  else
  {
    return false;
  }
}

int ControllerPrint()
{
  while (true)
  {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("pos= (%.1f  , %.1f  )       ", X, Y);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%.1f", gyro1.yaw(deg));
    if (devicesCheck())
    {
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print(":) Devices Connected         ");
    }
    else
    {
      Controller1.Screen.setCursor(3, 1);
      Controller1.rumble(".");
      if(!RotationB.installed()){
        Controller1.Screen.print("RotationB not connected");
      }
      else if (!RotationR.installed()){
        Controller1.Screen.print("RotationR not connected");
      }
      else if (!RotationL.installed()){
        Controller1.Screen.print("RotationL not connected");
      }
      else if (!LF.installed()){
        Controller1.Screen.print("LF not connected");
      }
      else if (!RF.installed()){
        Controller1.Screen.print("RF not connected");
      }
      else if (!LB.installed()){
        Controller1.Screen.print("LB not connected");
      }
      else if (!RB.installed()){
        Controller1.Screen.print("RB not connected");
      }
      else if (!gyro1.installed()){
        Controller1.Screen.print("gyro not connected");
      }
      else{
        Controller1.Screen.print("unknown device not connected");
      }
      wait(100, msec);
    }
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
  std::cout << "gyro calibrating" << std::endl;
  waitUntil(gyro1.isCalibrating() == false);
  if (devicesCheck())
  {
    Brain.Screen.clearScreen();
    Brain.Screen.print("Devices Connected");
  }
  else
  {
    Brain.Screen.clearScreen();
    Controller1.rumble("....");
    Brain.Screen.print("Devices Not Connected");
  }
  Brain.Screen.drawCircle(240, 120, 50, green);
  thread ControllerPrinting = thread(ControllerPrint);
  ControllerPrinting.setPriority(1);
 // int i = thread::hardware_concurrency();
 // std::cout <<"hardware limit =" <<i << std::endl;
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


  std::cout << "Autonomous Started yes" << std::endl;
  //driveAtAngle(-90,90,1);
  
 // wait(1,sec);
  float points[16][2] = {
      {0, 0},
      {5, 95},
      {-37,90 },
      {-41,90},

      {-41,90},
      {-45,90},
      {-85,90},
      {-90,90},

      {-90,90},
      {-95,90},
      {-113,84},
      {-113,60},

      {-113,60},
      {-113,36},
      {-113,30},
      {-113,-10}
      };
  stanley(points, sizeof(points) / (2*sizeof(float)));
  
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
      Brain.Screen.drawCircle(240, 120, 50, green);
    //  Controller1.rumble(".");

    voltDrive(Controller1.Axis3.position(), Controller1.Axis2.position(), 10);
        std::cout << X << ",,," << Y << std::endl;
    wait(15, msec); // Sleep the task for a short amount of time to
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
  //  Controller1.ButtonX.pressed(autonomous);

  Controller1.ButtonA.pressed(toggleclaw);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);

  }
}
