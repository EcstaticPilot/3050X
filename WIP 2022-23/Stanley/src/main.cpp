/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nikhi                                                     */
/*    Created:      3/14/2023, 3:29:14 PM                                     */
/*    Description:  Stanley Controller                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
extern double X,Y;
//drive.cpp
void drive(int lSpeed, int rSpeed, double wt);
void drive_brake(vex::brakeType Brake=brake);
void rotate(double dir, double accuracy = 1);
void inchDrive(double target, double speedMod=1);
void forward_dist(float dist);
void DriveToPoint(double targetX, double targetY, float speedMult = 1);
void RAMSETE(float targetX, float targetY, float targetAngle,float accuracy=1);
void DriveToPoint2(float targetX,float targetY);
void stanley(float points[][2],int length);
void purePursuit(float points[][2], int length);
//odometry
int odometery();


void toggleclaw(){
  claw.set(!claw.value());
}
int ControllerPrint() {


 
  while (1) {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("pos= (%.1f,%.1f)", X, Y);
     Controller1.Screen.setCursor(2, 1);
     Controller1.Screen.print("%.1f",gyro1.rotation(rotationUnits::deg));
    this_thread::sleep_for(75);
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

void pre_auton(void) {
  Brain.Screen.drawRectangle(0, 0, 480, 240, red);
  gyro1.calibrate();
  waitUntil(gyro1.isCalibrating()==false);

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

void autonomous(void) {
  Controller1.rumble("....");
  std::cout<<"Autonomous Started"<<std::endl;
  gyro1.calibrate();
  
  waitUntil(!gyro1.isCalibrating());
  //allocate memory space using malloc
  

  //assign x and y values for the points
  float points[42][2]={
{	0	,	0	},
{	0.32	,	9.00	},
{	0.53	,	17.28	},
{	0.64	,	24.86	},
{	0.65	,	31.77	},
{	0.56	,	38.03	},
{	0.38	,	43.66	},
{	0.11	,	48.70	},
{	-0.24	,	53.16	},
{	-0.67	,	57.08	},
{	-1.17	,	60.47	},
{	-1.75	,	63.37	},
{	-2.39	,	65.79	},
{	-3.09	,	67.77	},
{	-3.85	,	69.33	},
{	-4.66	,	70.49	},
{	-5.52	,	71.28	},
{	-6.43	,	71.73	},
{	-7.37	,	71.85	},
{	-8.36	,	71.69	},
{	-9.38	,	71.25	},
{	-10.42	,	70.57	},
{	-11.49	,	69.67	},
{	-12.58	,	68.58	},
{	-13.68	,	67.32	},
{	-14.79	,	65.92	},
{	-15.92	,	64.40	},
{	-17.04	,	62.79	},
{	-18.17	,	61.11	},
{	-19.28	,	59.39	},
{	-20.39	,	57.66	},
{	-21.48	,	55.93	},
{	-22.56	,	54.24	},
{	-23.61	,	52.61	},
{	-24.64	,	51.06	},
{	-25.63	,	49.63	},
{	-26.60	,	48.33	},
{	-27.52	,	47.19	},
{	-28.39	,	46.24	},
{	-29.22	,	45.50	},
{	-30.00	,	45.00	},
{	-29.99	,	45.01	}
  };
  stanley(points,sizeof(points)/8);
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
    std::cout<<X<<","<<Y<<std::endl;
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
int main() {
  // Set up callbacks for autonomous and driver control periods.
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
    Controller1.ButtonA.pressed(toggleclaw);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
