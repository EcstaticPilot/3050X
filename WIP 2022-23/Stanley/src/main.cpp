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
{	0.01	,	7.39	},
{	0.04	,	14.55	},
{	0.10	,	21.50	},
{	0.18	,	28.22	},
{	0.29	,	34.73	},
{	0.44	,	41.02	},
{	0.62	,	47.09	},
{	0.84	,	52.96	},
{	1.10	,	58.62	},
{	1.41	,	64.06	},
{	1.76	,	69.30	},
{	2.16	,	74.34	},
{	2.61	,	79.17	},
{	3.12	,	83.81	},
{	3.69	,	88.24	},
{	4.32	,	92.48	},
{	5.01	,	96.52	},
{	5.77	,	100.37	},
{	6.60	,	104.03	},
{	7.50	,	107.50	},
{	8.48	,	110.78	},
{	9.53	,	113.88	},
{	10.66	,	116.79	},
{	11.88	,	119.52	},
{	13.18	,	122.07	},
{	14.58	,	124.44	},
{	16.06	,	126.64	},
{	17.64	,	128.66	},
{	19.32	,	130.51	},
{	21.09	,	132.19	},
{	22.97	,	133.70	},
{	24.96	,	135.04	},
{	27.05	,	136.22	},
{	29.26	,	137.23	},
{	31.58	,	138.09	},
{	34.02	,	138.78	},
{	36.58	,	139.32	},
{	39.26	,	139.70	},
{	42.07	,	139.92	},
{	45.00	,	140.00	},
{45.01,140.00}
  
  };
  purePursuit(points,sizeof(points)/8);
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
