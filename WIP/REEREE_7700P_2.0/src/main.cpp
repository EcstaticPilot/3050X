/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*    Changes Made:                                                           */
/*    3/9/22 Abby added inchdrive, gyroturn, auton selecter, yellow rush      */
/*    3/11/22 Abby addded onto 120 Point Skills                                                                        */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LFDrive              motor         9               
// LBDrive              motor         8               
// LUDrive              motor         10              
// RFDrive              motor         2               
// RBDrive              motor         1               
// RUDrive              motor         3               
// Claw                 digital_out   A               
// Tilter               digital_out   B               
// Lift                 motor         7               
// Gyro                 inertial      6               
// backLift             motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"

using namespace vex;

competition Competition;


float d = 4.0; //Global Wheel Diameter
float pi = 3.1415926535897932384626;
float g = 7/5;

//GUI 

//CASE 0 = YELLOW RUSH
//CASE 1 = SKILLS
//CASE 2 = MIDDLE RUSH

int autonSelect = 3   ; //Default
int autonMin = 0;
int autonMax = 4;

//CLAW

//TRUE = OPEN
//FALSE = CLOSE

//GYRO

//RIGHT = 90
//LEFT = -90

void drawGUI() {
  // 2 buttons for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
  Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", autonSelect);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(20, 50, 100, 100);
  Brain.Screen.drawCircle(300, 75, 25);
  Brain.Screen.printAt(25, 75, "Select");
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(170, 50, 100, 100);
  Brain.Screen.printAt(175, 75, "GO");
  Brain.Screen.setFillColor(black);
}

void selectAuton() {
  bool selectingAuton = true;

  int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
  int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen
  // check to see if buttons were pressed
  if (x >= 20 && x <= 120 && y >= 50 && y <= 150) // select button pressed
  {
    autonSelect++;
    if (autonSelect > autonMax)autonSelect = autonMin; // rollover
      
    Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", autonSelect);
  }
  if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
    selectingAuton = false; // GO button pressed
    Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", autonSelect);
  }
  if (!selectingAuton) {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(300, 75, 25);
  } else {
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawCircle(300, 75, 25);
  }
  wait(10, msec); // slow it down
  Brain.Screen.setFillColor(black);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Gyro.calibrate();
  vexcodeInit();
  Brain.Screen.printAt(1, 40, "pre auton is running");
  drawGUI();
  Brain.Screen.pressed(selectAuton);
  Brain.Screen.printAt(20, 20, "LF Temp %f ", LFDrive.temperature(pct));
  Brain.Screen.printAt(20, 30, "LB Temp %f ", LBDrive.temperature(pct));
  Brain.Screen.printAt(20, 40, "LU Temp %f ", LUDrive.temperature(pct));
  Brain.Screen.printAt(20, 50, "RF Temp %f ", RFDrive.temperature(pct));
  Brain.Screen.printAt(20, 60, "RB Temp % f", RBDrive.temperature(pct));
  Brain.Screen.printAt(20, 70 , "RU Temp % f", RUDrive.temperature(pct));
  
}

void brakeDrive(){
  LFDrive.stop(brake);
  LBDrive.stop(brake);
  LUDrive.stop(brake);
  RFDrive.stop(brake);
  RBDrive.stop(brake);
  RUDrive.stop(brake);
}

void setHold(){
  LFDrive.setBrake(hold);
  LBDrive.setBrake(hold);
  LUDrive.setBrake(hold);
  RFDrive.setBrake(hold);
  RBDrive.setBrake(hold);
  RUDrive.setBrake(hold);
}

void setCoast(){
  LFDrive.setBrake(coast);
  LBDrive.setBrake(coast);
  LUDrive.setBrake(coast);
  RFDrive.setBrake(coast);
  RBDrive.setBrake(coast);
  RUDrive.setBrake(coast);
}

void inchDrive(float target, int speed, bool claw){

  float c = 0; //Current Location
  Claw.set(claw);
  LBDrive.setRotation(0, degrees);
  while (fabs(c) <= target) {
    LFDrive.spin(forward, speed, pct);
    LBDrive.spin(forward, speed, pct);
    LUDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
    RBDrive.spin(forward, speed, pct);
    RUDrive.spin(forward, speed, pct);
    c = LBDrive.rotation(rev) * pi * d * g;
  }

  brakeDrive();

}

void deploy(){
  Claw.set(true);
  Tilter.set(false);
}

void autonDriver(int wt, int lspeed, int rspeed, bool claw) //int liftspeed //bool claw //int Clawspin int moggs
{
  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct); 
  Claw.set(claw);
  
  /*//LLift.spin(forward, liftspeed, pct);
  //RLift.spin(forward, liftspeed, pct);
  ClawSpin.spin(forward, Clawspin, pct);
  mogolift.spin(forward, moggs, pct);
  wait(wt, msec);*/
}

void gyroTurn(float target) {
  while (Gyro.isCalibrating()) {
    // wait for Gyro Calibration , sleep but awwllow other tasks to run
    //90 = right, -90 = left
    this_thread::sleep_for(20);
  }
  float heading = 0;
  Gyro.setRotation(0, degrees);

  float speed = 0.0;
  float kp = 1.0;
  float d = 2.0;

  Brain.Screen.clearScreen();
  while (fabs(target - heading) >= d) {
    if (target - heading > 0) {
      speed = kp * (target - heading) + 10;
    }
    if (target - heading < 0) {
      speed = kp * (target - heading) - 10;
    }
    autonDriver(10, speed, -speed, false);;
    //Drive(10, speed, -speed);
    heading = Gyro.rotation(degrees);
    Brain.Screen.printAt(1, 40, "heading = %.3f", heading);
  }
  brakeDrive();
  //Brain.Screen.clearScreen();
}



//////////////////////////////////////////////////////////////////EOF//////////////////////////////////////////////////////////////////



void autonomous(void) { 

 //CLAW

 //TRUE = OPEN
 //FALSE = CLOSE

 //GYRO

 //RIGHT = 90
 //LEFT = -90

  switch (autonSelect) {

    case 0:

    //YELLOW RUSH

    
    deploy();
    inchDrive(32, 100, true);
    Claw.set(false);
    Lift.spin(reverse);
    inchDrive(35, -100, false);
    Lift.stop();
  
   //END OF YELLOW RUSH

   break;

    case 1:

    //SKILLS

    //120 POINTS RN

    deploy();
    backLift.setVelocity(100, pct);

    //red one

    inchDrive(118, 75, false);

    wait(500, msec);

   //yellow one next to really frikin big one in the middle if yk yk ;)

    gyroTurn(-90);
    wait(400, msec);
    inchDrive(26, 75, false);

    wait(200, msec);
    gyroTurn(-90);
    wait(400, msec);
  
    inchDrive(69, 75, false);
    wait(500, msec);
    inchDrive(6, -75, false);
    wait(500, msec);
  
   //yellow one in the middle            

    gyroTurn(90);
    wait(400, msec);
    inchDrive(48, 75, false);
    wait(500, msec);
    gyroTurn(90);
    wait(500, msec);
    inchDrive(57, 75, false);
    wait(500, msec);
    inchDrive(8, -75, false);
    wait(100, msec);

   //third yellow

    gyroTurn(-90);
    wait(100, msec);
    inchDrive(50, 75, false);
    wait(100, msec);
    gyroTurn(-90);
    wait(100, msec);
    inchDrive(70, 75, false);
    wait(500, msec);

   //red goal

    inchDrive(128, -75, false);
    wait(500, msec);
    gyroTurn(90);
    wait(500, msec);
    inchDrive(20, 75, false);
    wait(500, msec);
    gyroTurn(-90);
    wait(500, msec);
    inchDrive(130, 75, false);
    wait(500, msec);
  
   //blue one on red seesaw

    inchDrive(60, -100, false);
    wait(100, msec);
    gyroTurn(-87);
    wait(100, msec);
    inchDrive(120, 100, false);
    wait(100, msec);
    gyroTurn(90);
    wait(100, msec);
    backLift.spin(reverse);
    wait(1400, msec);
    backLift.stop(brake);
    inchDrive(50, -100, false);
    backLift.spin(forward);
    wait(1100, msec);
    backLift.stop(brake);
    wait(100, msec);
    inchDrive(100, 100, false); 
    wait(2000, msec);

   //END OF SKILLS
    break;
  

   //code 1

   
    case 2:
  
  // Sarah Skills
   
  deploy();
  inchDrive(60, 50, true);
  wait(200, msec);
  inchDrive(10, -50, true);
  wait(200, msec);
  gyroTurn(-70);
  inchDrive(10, 50, true);
  gyroTurn(-70);
  inchDrive(45, 50, true);

   //code 2

   break;

    case 3:

   // Middle Rush
    deploy();
    inchDrive(42, 100, true);
    Claw.set(false);
    Lift.spin(reverse);
    inchDrive(35, -100, false);
    Lift.stop();

   break;

  }
}


void usercontrol(void) {

   bool reversed = false;
   bool locked = false;
   bool aDown = 0; //Variable for when you're trying to reverse
   bool xDown = 0; //Variable for when you're locking the drive
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
    if(Controller1.ButtonX.pressing() && !xDown){
      locked = !locked;
      xDown = true;
    }
    else if (!Controller1.ButtonX.pressing()) {
      xDown = false;
    }
    if(locked){
      Brain.Screen.printAt(20, 160, "locked");
      setHold();
    }
    else if(!locked){
      Brain.Screen.printAt(20, 160, "coast");
      setCoast();
    }

    //Drive Code
    if(!reversed){
      LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      LUDrive.spin(forward, Controller1.Axis3.position(pct), pct);
      RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);
      RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
      RUDrive.spin(forward, Controller1.Axis2.position(pct), pct);
    }
    else if(reversed){
      LBDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      LFDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      LUDrive.spin(reverse, Controller1.Axis2.position(pct), pct);
      RBDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
      RFDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
      RUDrive.spin(reverse, Controller1.Axis3.position(pct), pct);
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
    if ((Controller1.ButtonDown.pressing() && !reversed) || (Controller1.ButtonL2.pressing() && reversed)){
      backLift.spin(reverse, 100, pct);
    }
    else if((Controller1.ButtonUp.pressing() && !reversed) || (Controller1.ButtonL1.pressing() && reversed)){
      backLift.spin(fwd, 100, pct);
    }
    else {
      backLift.stop(brake);
    }

    //Front Claw
    if (Controller1.ButtonR1.pressing()){
      Claw.set(true);
    }
    else if (Controller1.ButtonR2.pressing()){
      Claw.set(false);
    }

    //Tilter
    if (Controller1.ButtonY.pressing()){
      Tilter.set(true);
    }
    else if (Controller1.ButtonB.pressing()){
      Tilter.set(false);
    }
    wait(20, msec); 
                  
  }
}

int main() {
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  
  pre_auton();

  
  while (true) {
    wait(100, msec);
  }
}
