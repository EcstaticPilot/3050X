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
// RBDrive              motor         20              
// LBDrive              motor         11              
// LLift                motor         2               
// Controller1          controller                    
// LFDrive              motor         3               
// Gyro                 inertial      13              
// RLift                motor         9               
// Claw                 digital_out   A               
// RFDrive              motor         10              
// mogolift             motor         8               
// ClawSpin             motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
//Global Variable
float dia = 4.0;

//tis is GUI : Graphic User Interface


void Drive(int wt, int lspeed, int rspeed){
  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct);


}

void claw(bool claw){
  Claw.set(claw);
}

void lift(int liftspeed){
  LLift.spin(forward, liftspeed, pct);
  RLift.spin(forward, liftspeed, pct);
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

 void inchDrive (float target, int speed, bool claw) {
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
   autonDriver(0, 0, 0, 0, false);
    
}

void gyroTurn(float target, int Lspeed, int Rspeed, bool claw)
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
    autonDriver(10 , speed, -speed, 0, claw);
    wait(10,msec);
    heading4=Gyro.rotation(degrees); 
  }
  autonDriver(0, 0, 0, 0, true);
}
void breakdrive(){
  RBDrive.stop(brake);
  LBDrive.stop(brake);
  RFDrive.stop(brake);
  LFDrive.stop(brake);
}

void balance()
{
 float pitch=Gyro.pitch(deg);
 float oldpitch=pitch;
 inchDrive(10, 100, false);
     Brain.Screen.clearScreen();
     float kp=1;
     float kd = 20.0;

//float d=0.5;
while(true)//(fabs(pitch)>d)
{
  float speed = kp*pitch+kd*(pitch-oldpitch);
  Drive(speed, speed, 10);
  oldpitch=pitch;
    pitch = Gyro.pitch(deg);
    Brain.Screen.printAt(1, 100, "pitch=   %.3f   ",pitch);

}
breakdrive();
Brain.Screen.printAt(1, 150, "i am done ");

}







////////////----------------------EOF-----------------------//////////////////////

void pre_auton(void) {
wait(2000, msec);
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
 //claw true is closed, false is open
 //happy face
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
 //printing some words :)
 Brain.Screen.setFillColor(green);
 Brain.Screen.printAt(20, 130,"7700P is better than" );
 Brain.Screen.printAt(20, 150, "7700E and thats a fact");

//Abby's Code Below
 //auton
 /*inchDrive(2, 100,  false);
 inchDrive(4, -100, false);
 inchDrive(0, 50, false);
 wait(500, msec);
 inchDrive(118, 75, false);
 wait(500, msec);
 autonDriver(36, -100, 100, 0, false);
 wait(400, msec);
 inchDrive(24, 75, false);
 autonDriver(45, -100, 120, 0, false);
 wait(400, msec);
 inchDrive(100, 75, false);
 wait(500, msec);
 //big one in the middle
 inchDrive(15, -75, false);
 wait(200, msec);
 autonDriver(27, 100, -100, 0, false);
 wait(400, msec);
 inchDrive(40, 74, false);
 wait(500, msec);
 autonDriver(36, 100, -100, 0, false);
 wait(400, msec);
 inchDrive(104, 75, false);
 wait(500, msec);
 autonDriver(36, -100, 100, 0, false);
 wait(400, msec);
 inchDrive(22, 75, false);

 //3rd yellow goal
 autonDriver(36, -100, 100, 0, false);
 inchDrive(118, 75, false);
 
 //last blue goal
 inchDrive(100, -75, false);
 
 //mogo part
 //gets last blue and pulls it to other side
 mogolift.spin(reverse, 100, pct);
 wait(600, msec);
 mogolift.stop();
 inchDrive(5, -75, false);
 mogolift.spin(forward, 100, pct);
 wait(700, msec);
 mogolift.stop();
 inchDrive(98, 75, false);
 mogolift.spin(reverse, 100, pct);
 wait(700, msec);
 mogolift.stop();
 */

 //Felix Code
 float backLiftTime = 1200;
//Grab the seesaw mogo
 mogolift.setVelocity(100, percent);
mogolift.spin(reverse);
wait(backLiftTime, msec);
mogolift.stop(brake);
inchDrive(9, -75, false);
mogolift.spin(forward);
wait(backLiftTime,msec);
mogolift.stop(brake);
//Go for the left yellow mogo
inchDrive(6, 100, false);
gyroTurn(15, 75, -75, false);
inchDrive(3, 75, false);
gyroTurn(45, 75, -75, false);
inchDrive(75, 75, false);
inchDrive(35, -75, false );
//Go for the middle yellow mogo
gyroTurn(20, 75, -75, false);
inchDrive(70, 100, false);
//Go for the right mogo
inchDrive(15, 100, false);
gyroTurn(15, 75, -5, false);
inchDrive(5, 100, false);
gyroTurn(200, 75, -75, false);
inchDrive(40, 100, false);
balance();

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
     ClawSpin.setVelocity(100, percent);
     

     
   Brain.Screen.printAt(160, 60, "hi");
Brain.Screen.printAt( 20, 20,"LB Motor Temp%f ",LBDrive.temperature(pct));
Brain.Screen.printAt( 20, 40,"RB Motor Temp%f ",RBDrive.temperature(pct));
Brain.Screen.printAt( 20, 60,"LF Motor Temp%f ",LFDrive.temperature(pct));
Brain.Screen.printAt( 20, 80,"RF Motor Temp%f ",RFDrive.temperature(pct));

  LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
  RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
  RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);

if (Controller1.ButtonX.pressing())
{
    LFDrive.setBrake(hold);
    LBDrive.setBrake(hold);
    RFDrive.setBrake(hold);
    RBDrive.setBrake(hold);
}
else{
      LFDrive.setBrake(coast);
    LBDrive.setBrake(coast);
    RFDrive.setBrake(coast);
    RBDrive.setBrake(coast);
}

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


 }


   if (Controller1.ButtonLeft.pressing()) {
    ClawSpin.spin(forward, 20, pct);


    
   }
   else if 
     (Controller1.ButtonRight.pressing()) {
      ClawSpin.spin(reverse, 20, pct);
       
     }
     else{
       ClawSpin.stop(hold);
     }
 
 if(Controller1.ButtonUp.pressing()){
   mogolift.spin(forward, 100, pct);
 }
 else if(Controller1.ButtonDown.pressing()){
   mogolift.spin(reverse, 100, pct);
 }
 else{
   mogolift.stop(hold);
 }
 
 }




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

