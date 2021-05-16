/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\chiep                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightBackMotor       motor         5               
// LeftBackMotor        motor         6               
// Arm                  motor         12              
// Controller1          controller                    
// Lintake              motor         16              
// Rintake              motor         18              
// Lfront               motor         1               
// Rfront10             motor         10              
// Inertial4            inertial      4               
// Arm2                 motor         20              
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
Brain.Screen.printAt( 10 , 20,"RMotor Temp%f ",LeftBackMotor.temperature(pct));
Brain.Screen.printAt( 10, 35,"LMotor Temp%f ",RightBackMotor.temperature(pct));
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


void Drive(float speed)
{
     LeftBackMotor.spin(forward,speed, pct);
    RightBackMotor.spin(forward, speed, pct);
    Lfront.spin(forward, speed, pct);
    Rfront10.spin(forward, speed, pct);
}

void Pdrive (float target, float distrav)
{
float error = 0;
float Dist = 0;
float kP = 1;
while(true) {
Dist = Lfront.position(degrees),( 4*3.14/360);
error = target - Dist;
Drive(error*kP);
wait(5,msec);

}
}


void autonDriver(int wt, int Lspeed, int Rspeed, int Armspeed, int Lint, int Rint) 
{

  LeftBackMotor.spin(forward, Lspeed, pct);
  RightBackMotor.spin(forward, Rspeed, pct);
  Lfront.spin(forward, Lspeed, pct);
  Rfront10.spin(forward, Rspeed, pct); 
  Arm.spin(forward, Armspeed, pct);
  Arm2.spin(forward, Armspeed, pct);
  Lintake.spin(forward, Lint, pct);
  Rintake.spin(forward, Rint, pct);

wait(wt, msec);
}

 void InchDrive (float target, int speed) {

  float c = 0; //our distance
LeftBackMotor.setRotation(0, degrees);
  while (fabs (c) <= target) {

    LeftBackMotor.spin(forward,speed, pct);
    RightBackMotor.spin(forward, speed, pct);
    Lfront.spin(forward, speed, pct);
    Rfront10.spin(forward, speed, pct);
    
   c = LeftBackMotor.rotation(rev)*3.14*dia; 
  }
   autonDriver(0, 0, 0, 0, 0, 0);
    
}

void gyroTurn(float target, int Lspeed, int Rspeed)
{
/*  while(Inertial4.isCalibrating())
{
  // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    this_thread::sleep_for(20);
}*/
  float heading4 = 0;
  Inertial4.setRotation(0, degrees); 
 
float speed=0.0 ;
float kp = 2.0;
  while (fabs(heading4)<= target) 
{
    speed=kp*(target-heading4);
    autonDriver(10 , Lspeed, Rspeed, 0, 0, 0);
    wait(10,msec);
    heading4=Inertial4.rotation(degrees); 
  }
  autonDriver(0, 0, 0, 0, 0, 0);
}
////////////----------------------EOF-----------------------//////////////////////

void pre_auton(void) {

}

 void autonomous(void) {
    while(Inertial4.isCalibrating())
{
  
    wait(20,msec);
}
 //Brain.Screen.printAt(160, 60, "hi");
Brain.Screen.printAt( 20, 20,"Motor Temp%f ",LeftBackMotor.temperature(pct));
Brain.Screen.printAt( 20, 40,"Motor Temp%f ",RightBackMotor.temperature(pct));
Brain.Screen.printAt( 20, 60,"Motor Temp%f ",Lfront.temperature(pct));
Brain.Screen.printAt( 20, 80,"Motor Temp%f ",Rfront10.temperature(pct));
 Brain.Screen.printAt( 20, 100,"Heading%f "   , Inertial4.rotation(deg));

//Brain.Screen.printAt( 20, 100,"Heading%f ",  Intertial4.setRotation(deg));



//scoring in the corner goal
autonDriver(200, -100, -100, 0, 90, 90); //intake + backwards
autonDriver(100, 0, 0, 0, 0, 0); //pause
autonDriver(2000, 0, 0, 85, 0, 0); //arm
autonDriver(1200, 50, 50, 20, 0, 0); //forward + arm
autonDriver(210, -50, 50, 20, 0, 0); //turn
autonDriver(500, 10, 10, 20, 0, 0); //forward slightly
autonDriver(600, 0, 0, 20, 80, 80); //outake
autonDriver(500, -50, -50, 20, 0, 0); //backwards
autonDriver(500, 0, 0, 20, 0, 0); //pause and lift

//face the ball in the middle
gyroTurn(145, 20, -20);

autonDriver(1700, 70, 70, -90, -100, -100);//Forward + arm down + intake
autonDriver(100, -5, -5, 0, 0, 0);

gyroTurn(55, -20, 20);

for(int i = 0; i < 3; i++)
{
autonDriver(650, 60, 60, -5, 0, 0); //forward
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(600, -60, -60, -5, 0, 0); //backwards
autonDriver(50, 0, 0, 0, 0, 0); //pause
}

autonDriver(800, 5, -5, 100, -10, -10);
InchDrive(12, 20);
autonDriver(700, 0, 0, 10, 70, 70);

/*
//autonDriver(1000, -52, 50, 20, 0, 0); //turn
autonDriver(200, 0, 0, 0, 0, 0); //pause
autonDriver(1600, 85, 85, -80, 0, 0); //forward
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(1000, -18, 30, -5, 0, 0); //turn
autonDriver(200, 0, 0, 0, 0, 0); //pause
autonDriver(600, 50, 50, -5, 0, 0); //forward
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(500, -50, -50, -5, 0, 0); //backwards
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(600, 50, 50, 0, 0, 0); //forward
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(500, -50, -50, 0, 0, 0); //backwards
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(600, 50, 50, 0, 0, 0); //forward
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(500, -50, -50, 0, 0, 0); //backwards
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(600, 50, 50, 0, 0, 0); //forward
autonDriver(50, 0, 0, 0, 0, 0); //pause
autonDriver(500, -50, -50, 0, 0, 0); //backwards
autonDriver(50, 0, 0, 0, 0, 0); //pause

//Next phase

autonDriver(300, 90, -90, 0, 0, 0); //turn right
autonDriver(50, 0, 0, 0, -90, 90); //pause
autonDriver(700, 75, 75, 0, -90, -90); //forward+intake
autonDriver(2000, 0, 0, 85, -90, -90); //lift up
autonDriver(750, -49, 49, 20, 0, 0); //turn left
autonDriver(70, 10, -10, 20, -90, -90); //pause
autonDriver(450, 50, 50, 20, 0, 0); //forward
autonDriver(300, 0, 0, 20, 70, 70); //outake
autonDriver(500, -70, -70, 20, 0, 0); //backward

 autonDriver(100, 35, -35, 0, -90, -90); //turn right slightly
autonDriver(1300, 60, 60, 0, -90, -90); //forward
autonDriver(2000, 0, 0, 85, -90, -90); //lift up
autonDriver(200, -85, 85, 0, 0, 0); //turn left
autonDriver(700, 60, 60, 0, -90, -90); //forward
autonDriver(700, 0, 0, 20, 90, 90); //outake

autonDriver(500000, 0, 0, 0, 0, 0); //End
*/

}

  

 void driverControl () {

   while (true){
     motorCurrent(); 
   Brain.Screen.printAt(160, 60, "hi");
Brain.Screen.printAt( 20, 20,"Motor Temp%f ",LeftBackMotor.temperature(pct));
Brain.Screen.printAt( 20, 40,"Motor Temp%f ",RightBackMotor.temperature(pct));
Brain.Screen.printAt( 20, 60,"Motor Temp%f ",Lfront.temperature(pct));
Brain.Screen.printAt( 20, 80,"Motor Temp%f ",Rfront10.temperature(pct));

  LeftBackMotor.spin(forward, Controller1.Axis3.position(pct), pct);
  RightBackMotor.spin(forward, Controller1.Axis2.position(pct), pct);
  Lfront.spin(forward, Controller1.Axis3.position(pct), pct);
  Rfront10.spin(forward, Controller1.Axis2.position(pct), pct);


  if (Controller1.ButtonR1.pressing()) {
    Lintake.spin(forward, 100, pct);
    Rintake.spin(forward, 100, pct);
    
    
   }
   else if 
     (Controller1.ButtonR2.pressing()) {
       Lintake.spin(reverse, 100, pct);
       Rintake.spin(reverse, 100, pct);
       
     }
  
 
 else {
   Lintake.stop(brake);
   Rintake.stop(brake);
   


 }
 if (Controller1.ButtonL1.pressing()) {
    Arm.spin(forward, 85, pct);
    Arm2.spin(forward, 85, pct);

    
   }
   else if 
     (Controller1.ButtonL2.pressing()) {
       Arm.spin(reverse, 85, pct);
       Arm2.spin(reverse, 85, pct);
     }
  
 
 else {
       Arm.stop(brake);
       Arm2.stop(brake);


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

