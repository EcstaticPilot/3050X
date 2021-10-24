/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      Sat Jul 17 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftBack             motor         11              
// LeftFront            motor         1               
// RightBack            motor         20              
// RightFront           motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

void pre_auton()
{

}

void Competition()
{

}



void autonDriver(int wt, int lspeed, int rspeed, int liftspeed, bool claw) 
{

  LeftBackDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct); 
  LLift.spin(forward, liftspeed, pct);
  RLift.spin(forward, liftspeed, pct);
  Claw.set(claw);

wait(wt, msec);
}

 void InchDrive (float target, int speed, int arm,bool claw) {
Claw.set(claw);
  float c = 0; //our distance
LBDrive.setRotation(0, degrees);
  while (fabs (c) <= target) {

    LBDrive.spin(forward,speed, pct);
    RBDrive.spin(forward, speed, pct);
    LFDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
    
   c = LBDrive.rotation(rev)*3.14*dia; 
  }
   autonDriver(0, 0, 0, arm, false);
    
}
}

void driver(){
  
  Brain.Screen.printAt(1,  40,  "Hello  Friends Driver is Running");
  // User control code here, inside the loop
  while (true) {
    LeftBack.spin(forward, Controller1.Axis3.position(),pct); 
    LeftFront.spin(forward, Controller1.Axis3.position(),pct); 
    RightBack.spin(forward, Controller1.Axis2.position(), pct); 
    RightFront.spin(forward, Controller1.Axis2.position(), pct);
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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }

}
