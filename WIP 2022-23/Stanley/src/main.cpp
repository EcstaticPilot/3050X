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

void toggleclaw() { claw.set(!claw.value()); }

button red1;
button red2;
button blue1;
button blue2;

enum Auton {RED_1, RED_2, BLUE_1, BLUE_2};

Auton selectedAuton = RED_1;

void onScreenPress()
{
  if (red1.checkTouch())
  {
    red1.setFill(true);
    red2.setFill(false);
    blue1.setFill(false);
    blue2.setFill(false);

    selectedAuton = RED_1;
  }
  else if (red2.checkTouch())
  {
    red1.setFill(false);
    red2.setFill(true);
    blue1.setFill(false);
    blue2.setFill(false);

    selectedAuton = RED_2;
  }
  else if (blue1.checkTouch())
  {
    red1.setFill(false);
    red2.setFill(false);
    blue1.setFill(true);
    blue2.setFill(false);

    selectedAuton = BLUE_1;
  }
  else if (blue2.checkTouch())
  {
    red1.setFill(false);
    red2.setFill(false);
    blue1.setFill(false);
    blue2.setFill(true);
    
    selectedAuton = BLUE_2;
  }
}

bool devicesCheck()
{

  if (LF.installed() && RF.installed() && LB.installed() && RB.installed() &&
      gyro1.installed()
      )
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
    Controller1.Screen.print("%f", gyro1.angle(degrees));
    Controller1.Screen.setCursor(3, 1);
    if (devicesCheck())
    {
      Controller1.Screen.print(":) Devices Connected         ");
    }
    else
    {
      Controller1.rumble(".");
      if (!LF.installed())
      {
        Controller1.Screen.print("LF not connected");
      }
      else if (!RF.installed())
      {
        Controller1.Screen.print("RF not connected");
      }
      else if (!LB.installed())
      {
        Controller1.Screen.print("LB not connected");
      }
      else if (!RB.installed())
      {
        Controller1.Screen.print("RB not connected");
      }
      else if (!gyro1.installed())
      {
        Controller1.Screen.print("gyro not connected");
      }
      else
      {
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
//  red1 =  button(60, 60, 120, 60, red, "red1");
 // red2 =  button(60, 130, 120, 60, red, "red2");
 // blue1 = button(300, 60, 120, 60, blue, "blue1");
 // blue2 = button(300, 130, 120, 60, blue, "blue2");

 // Brain.Screen.pressed(onScreenPress);

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

  //launch threads
  thread ControllerPrinting = thread(ControllerPrint);
  ControllerPrinting.setPriority(1);
  thread posTrack = thread(odometery);

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

  float throughGoal[16][2] = {
      {0, 0},
{0, 120},
{-10, 120},
{35, 120},
// mirror over 36.5
{35, 120},
{80, 120},
{68, 120},
{68, 60},

{68,60},
{68,0},
{80,0},
{34,0},

{34,0},
{-10,0},
{0,0},
{0,60}};
  stanley(throughGoal, sizeof(throughGoal) / (2 * sizeof(float)));
  wait(0.5, sec);


 switch (selectedAuton)
  {
  case RED_1:
    // code for red auton 1
    break;
  case RED_2:
    // code for red auton 2
    break;
  case BLUE_1:
    // code for blue auton 1
    break;
  case BLUE_2:
    // code for blue auton 2
    break;
  default:
    
    break;
    
  }
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
    std::cout << X << ",,," << Y << "\n";
    wait(15, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{

  // Set up callbacks for autonomous and driver control periods.
  // Competition.bStopAllTasksBetweenModes = true;
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
