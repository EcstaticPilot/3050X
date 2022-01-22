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
// RBDrive              motor         10              
// LBDrive              motor         11              
// Controller1          controller                    
// LFDrive              motor         20              
// RFDrive              motor         13              
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




////////////----------------------EOF-----------------------//////////////////////

 void driverControl () {
  LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
  RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);
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



