/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      Fri Apr 15 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Lift                 motor         5               
// RFDrive              motor         10              
// RMDrive              motor         9               
// RBDrive              motor         8               
// LFDrive              motor         1               
// LMDrive              motor         2               
// LBDrive              motor         3               
// Controller1          controller                    
// Claw                 digital_out   A               
// Tilter               digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;


void usercontrol(void) {

   bool reversed = false;
   bool tiltedUp = false;
   bool aDown = 0; //Variable for when you're trying to reverse
   bool bDown = 0; //Variable for when you're locking the drive
   Lift.setBrake(hold);

  while (1) {
    //Reversing Drive
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
    
    //Locking Drive
    if(Controller1.ButtonB.pressing() && !bDown){
      tiltedUp = !tiltedUp;
      bDown = true;
    }
    else if (!Controller1.ButtonB.pressing()) {
      bDown = false;
    }
    if(tiltedUp){
      Tilter.set(true);
    }
    else if(!tiltedUp){
      Tilter.set(false);
    }

    //Drive Code
    if(!reversed){
      LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      LMDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);
      RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
      RMDrive.spin(forward, Controller1.Axis2.position(pct), pct);
    }
    else if(reversed){
      LBDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      LFDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      LMDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      RBDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
      RFDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
      RMDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
    }
      
      //Lift Code
    if((Controller1.ButtonL1.pressing() && !reversed) || (Controller1.ButtonUp.pressing() && reversed)){
      Lift.spin(fwd, 100, pct);
    }
    else if((Controller1.ButtonL2.pressing() && !reversed) || (Controller1.ButtonDown.pressing() && reversed)){
      Lift.spin(reverse, 100, pct);
    }
    else{
      Lift.stop(brake);
    }

    //Back Lift 
    /*
    if ((Controller1.ButtonDown.pressing() && !reversed) || (Controller1.ButtonL2.pressing() && reversed)){
      backLift.spin(reverse, 100, pct);
    }
    else if((Controller1.ButtonUp.pressing() && !reversed) || (Controller1.ButtonL1.pressing() && reversed)){
      backLift.spin(fwd, 100, pct);
    }
    else {
      backLift.stop(brake);
    }
    */

    //Front Claw
    if (Controller1.ButtonR1.pressing()){
      Claw.set(true);
    }
    else if (Controller1.ButtonR2.pressing()){
      Claw.set(false);
    }

    //Locking Drive
    if (Controller1.ButtonX.pressing()){
      setHold();
    }
    if (Controller1.ButtonY.pressing()){
      setCoast();
    }

    wait(20, msec); 
                  
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
