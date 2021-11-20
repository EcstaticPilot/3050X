/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\chiep                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  9-1-21 added pneumatics                                              */
/*                   abby added LEFT auton 9/17/21                                                         */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RBDrive              motor         18              
// LBDrive              motor         10              
// Controller1          controller                    
// LFDrive              motor         14              
// RFDrive              motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
//Global Variable
float dia = 4.0;

//tis is GUI : Graphic User Interface


void Drive(int lspeed, int rspeed){
  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct);
}

void draw(){

Brain.Screen.setFillColor(red);
Brain.Screen.setPenColor(white);
Brain.Screen.drawRectangle(10, 10, 100, 50);
Brain.Screen.printAt(10, 10, "hi");
Brain.Screen.printAt( 10 , 20,"RMotor Temp%f ",LBDrive.temperature(pct));
Brain.Screen.printAt( 10, 35,"LMotor Temp%f ",RBDrive.temperature(pct));
//Brain.Screen.printAt(10, 40,"Motor");


Brain.Screen.setFillColor(blue);
Brain.Screen.setPenColor(white);
Brain.Screen.drawRectangle(400, 10, 80, 50);
Brain.Screen.printAt(410, 15, "C++" );



}

void motorCurrent() { 
 /* Brain.Screen.printAt( 10, 80,"LMotor current%f  ",LeftBackMotor.current());
  Brain.Screen.printAt( 10, 96,"RMotor current%f  ",RightBackMotor.current());
  Brain.Screen.printAt( 10, 110,"LMotor current%f ",Lfront.current());
  Brain.Screen.printAt( 10, 122,"RMotor current%f ",Rfront10.current());
  Brain.Screen.printAt( 10, 132,"Motor current%f  ",Arm.current());
  */



}


////////////----------------------EOF-----------------------//////////////////////

 void driverControl () {
   Drive(10, 10);
     Brain.Screen.setFillColor(yellow);
 Brain.Screen.drawCircle(360, 125, 110);
 Brain.Screen.setPenColor(black);
 Brain.Screen.drawLine(315, 180, 400, 180);
 Brain.Screen.drawLine(315, 181, 400, 181);
 Brain.Screen.drawLine(315, 182, 400, 182);
 Brain.Screen.drawLine(315, 183, 400, 183);
 //end of first line
 Brain.Screen.drawLine(290, 100, 350, 100);
 Brain.Screen.drawLine(290, 101, 350, 101);
 Brain.Screen.drawLine(290, 102, 350, 102);
 Brain.Screen.drawLine(290, 103, 350, 103);
 //end of second line
 Brain.Screen.drawLine(375, 100, 430, 100);
 Brain.Screen.drawLine(375, 101, 430, 101);
 Brain.Screen.drawLine(375, 102, 430, 102);
 Brain.Screen.drawLine(375, 103, 430, 103);

  LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
  RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);

if (Controller1.ButtonX.pressing())
{
    LFDrive.setBrake(brake);
    LBDrive.setBrake(brake);
    RFDrive.setBrake(brake);
    RBDrive.setBrake(brake);
}
else{
      LFDrive.setBrake(coast);
    LBDrive.setBrake(coast);
    RFDrive.setBrake(coast);
    RBDrive.setBrake(coast);
}
 }

 int main() {
  vexcodeInit();
  Competition.drivercontrol(driverControl);
while (true){
wait (100, msec); 
}
    // Run the pre-autonomous function.
  }
  // Initializing Robot Configuration. DO NOT REMOVE!



