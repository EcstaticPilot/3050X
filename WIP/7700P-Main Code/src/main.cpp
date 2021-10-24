/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\chiep    a                                        */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  9-1-21 added pneumatics
//main code
//CURRENT AUTON 9/24/21 */
/*                                                                            */
/*-----------------------------           ccd-----------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RBDrive              motor         20              
// LBDrive              motor         11              
// LLift                motor         2               
// Controller1          controller                    
// LFDrive              motor         1               
// Gyro                 inertial      13              
// RLift                motor         9               
// Claw                 digital_out   A               
// RFDrive              motor         10              
// ClawSpin             motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
//Global Variable
float dia = 4.0;

//tis is GUI : Graphic User Interface
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

void Drive(float speed)
{
     LBDrive.spin(forward,speed, pct);
    RBDrive.spin(forward, speed, pct);
    LFDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
}

void Pdrive (float target, float distrav)
{
float error = 0;
float Dist = 0;
float kP = 1;
while(true) {
Dist = LFDrive.position(degrees),( 4*3.14/360);
error = target - Dist;
Drive(error*kP);
wait(5,msec);

}
}


void autonDriver(int wt, int lspeed, int rspeed, int liftspeed, bool claw) 
{

  LBDrive.spin(forward, lspeed, pct);
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
   autonDriver(0, 0, 0, arm, claw);
    
}

void lift(int liftspeed){
  LLift.spin(forward, liftspeed, pct);
  RLift.spin(forward, liftspeed, pct);
}

void gyroTurn(float target, int Lspeed, int Rspeed, int lift, bool claw)
{
  while(Gyro.isCalibrating())
{
  // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    this_thread::sleep_for(20);
}
  float heading4 = 0;
  Gyro.setRotation(0, degrees); 
 
float speed=0.0 ;
float kp = 2.0;
  while (fabs(heading4)<= target) 
{
    speed=kp*(target-heading4);
    autonDriver(10 , Lspeed, Rspeed, lift, claw);
    wait(10,msec);
    heading4=Gyro.rotation(degrees); 
  }
  autonDriver(0, 0, 0, lift, claw);
}
////////////----------------------EOF-----------------------//////////////////////

void pre_auton(void) {
 //Claw.set(true);
}

 void autonomous(void) {
    while(Gyro.isCalibrating())
{
  
    wait(20,msec);
}
// autonDriver(wait time, left drive speed, right drive speed, lift speed, claw close is true) 
Brain.Screen.printAt( 20, 20,"Motor Temp%f ",LBDrive.temperature(pct));
Brain.Screen.printAt( 20, 40,"Motor Temp%f ",RBDrive.temperature(pct));
Brain.Screen.printAt( 20, 60,"Motor Temp%f ",LFDrive.temperature(pct));
Brain.Screen.printAt( 20, 80,"Motor Temp%f ",RFDrive.temperature(pct));
 Brain.Screen.printAt( 20, 100,"Heading%f ",Gyro.rotation(deg));

//Brain.Screen.printAt( 20, 100,"Heading%f ",  Intertial4.setRotation(deg));
//true is open, false is close


//win point
InchDrive(1, 60, 0, false);
InchDrive(3, -50, 0, false);
InchDrive(7, 50, 0, false);
InchDrive(1, 2, 0, true);
wait(500, msec);
//pulling ring w/ goal back

InchDrive(0,0,0,false);
wait(100,msec);
InchDrive(15, -50,0,false);




}

  

 void driverControl () {

   while (true){
     if (Controller1.ButtonR2.pressing())
     {
       Claw.set(true);
     }
     else if(Controller1.ButtonR1.pressing())
     {
       Claw.set(false);
     }
    while (true){
      if (Controller1.ButtonY.pressing())
      {
        ClawSpin.spin(forward, 100, pct);
        
      }
      else if(Controller1.ButtonA.pressing())
      {
        ClawSpin.spin(reverse, 100, pct);
      }
    }

     
   Brain.Screen.printAt(160, 60, "hi");
Brain.Screen.printAt( 20, 20,"LB Motor Temp%f ",LBDrive.temperature(pct));
Brain.Screen.printAt( 20, 40,"RB Motor Temp%f ",RBDrive.temperature(pct));
Brain.Screen.printAt( 20, 60,"LF Motor Temp%f ",LFDrive.temperature(pct));
Brain.Screen.printAt( 20, 80,"RF Motor Temp%f ",RFDrive.temperature(pct));

  LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
  RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);

 if (Controller1.ButtonL1.pressing()) {
    LLift.spin(forward, 100, pct);
    RLift.spin(forward, 100, pct);

    
   }
   else if 
     (Controller1.ButtonL2.pressing()) {
       LLift.spin(reverse, 100, pct);
       RLift.spin(reverse, 100, pct);
     }
  
 
 else {
       LLift.stop(brake);
       RLift.stop(brake);


 }}
  }






int main() {
  vexcodeInit();
  
Competition.autonomous(autonomous);
  Competition.drivercontrol(driverControl);


    pre_auton();
   
   
    //autonomous ();
    //driverControl(); 
while (true){
wait (100, msec); 
}
    // Run the pre-autonomous function.
  }
  // Initializing Robot Configuration. DO NOT REMOVE!

