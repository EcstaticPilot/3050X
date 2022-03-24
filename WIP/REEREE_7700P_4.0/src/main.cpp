/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*    Changes Made:                                                           */
/*                                                                            */
/*    3/9/22 Abby added inchdrive, gyroturn, auton selecter, yellow rush      */
/*    3/11/22 Abby addded onto 120 Point Skills                               */
/*    3/12/22 Made 80 point programming skills for the new robot, made the    */
/*            tilter a toggle and the locking drive no longer a toggle        */
/*    3/13/22 At comp abby, changed skills, finalized yellow rush (dont touch)*/
/*    inch drive and gyro turn need to be fixed                               */
/*    55 degrees for gyro = 90 and 30 inches for inch drive = whole field     */
/*    3/15/22 Abby fixed middle + yellow rush and pushing skills              */
/*    started working on lifting skills (lifting goal onto platforms)         */
/*    instead of just pushing goals                                           */
/*    3/22/22 Abby worked on lifting skills (160 points max) 200 if balance   */
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

int autonSelect = 7 ; //Default
int autonMin = 0;
int autonMax = 8;

//GUI:

  //CASE 0 = YELLOW RUSH
  //CASE 1 = ABBY 100 POINT PUSHING KILLS
  //CASE 2 = 80 POINT SEAN SKILLS
  //CASE 3 = MIDDLE RUSH
  //CASE 4 = DO NOTHING AUTON
  //CASE 5 = YELLOW RUSH + MIDDLE RUSH
  //CASE 6 = 40 POINT LIFT (EMERGENCY USE)
  //CASE 7 = ABBY LIFTING SKILLS


//CLAW:

  //TRUE = OPEN
  //FALSE = CLOSE

//GYRO:

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
void lift_up(int speed){
  Lift.spin(forward, speed, pct);
  wait(500, msec);
  //Lift.stop();


}
void lift_down(int speed){
  Lift.spin(reverse, speed, pct);
  wait(500, msec);
  Lift.stop();


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

void Drive(int wt, int lspeed, int rspeed,
           bool driveVolts = false) { // bool = optional var
  if (driveVolts == true) {
    lspeed*=120;
    lspeed*=120;
    LBDrive.spin(forward, lspeed , voltageUnits::mV);
    LFDrive.spin(forward, lspeed , voltageUnits::mV);
    RBDrive.spin(forward, rspeed , voltageUnits::mV);
    RFDrive.spin(forward, rspeed , voltageUnits::mV);
    RUDrive.spin(forward, rspeed, voltageUnits::mV);
    LUDrive.spin(forward, rspeed, voltageUnits::mV);

  } else {

    LBDrive.spin(forward, lspeed, pct);
    RBDrive.spin(forward, rspeed, pct);
    LFDrive.spin(forward, lspeed, pct);
    RFDrive.spin(forward, rspeed, pct);
    RUDrive.spin(forward, rspeed, pct);
    LUDrive.spin(forward, lspeed, pct);
  }
  wait(wt, msec);
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
  float kp = 1.0; //1
  float d = 2.0;

  Brain.Screen.clearScreen();
  while (fabs(target - heading) >= d) {
    if (target - heading > 0) {
      speed = kp * (target - heading) + 10;
    }
    if (target - heading < 0) {
      speed = kp * (target - heading) - 10;
    }
    //autonDriver(10, speed, -speed, false);;
    Drive(10, speed, -speed);
    heading = Gyro.rotation(degrees);
    Brain.Screen.printAt(1, 40, "heading = %.3f", heading);
  }
  brakeDrive();
  //Brain.Screen.clearScreen();
}
void balance()
{
 float pitch=Gyro.pitch(deg);
 float oldpitch=pitch;
 inchDrive(10, 55,true);
     Brain.Screen.clearScreen();
     float kp=1;
      float kd = 15.0;

//float d=0.3;
while(true)//(fabs(pitch)>d)
{
  float speed = kp*pitch+kd*(pitch-oldpitch);
  Drive(speed, -speed, 10);
 
  oldpitch=pitch;
    pitch = Gyro.pitch(deg);
    Brain.Screen.printAt(1, 100, "pitch=   %.3f   ",pitch);

}
brakeDrive();
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

    case 0: //YELLOW RUSH
  
    deploy();
    inchDrive(32, 75, true);
    Claw.set(false);
    wait(100, msec);
    Lift.spin(reverse);
    inchDrive(35, -75, false);
    Lift.stop();
    break;

    case 1: //ABBY SKILLS
    //robot is much faster, changing speed to 50 instead of 75
    //diving inch drive by half
   inchDrive(15, 75, false);
    balance();


    /*deploy();
    backLift.setVelocity(75, pct);
    inchDrive(59, 50, false); //Red Mogo //118
    wait(300, msec); //Short Neutral Goal

    gyroTurn(-80); //left turn
    wait(300, msec);
    inchDrive(13, 55, false); //26

    wait(300, msec);
    gyroTurn(-80);
    wait(300, msec);
  
    inchDrive(34, 55, false); //og 69 
    wait(300, msec);
    inchDrive(33, -55, false); //46
    wait(300, msec);

    gyroTurn(80); //Tall Neutral Goal
    wait(300, msec);
    inchDrive(20, 55, false); //48
    wait(300, msec);
    gyroTurn(-80);
    wait(300, msec);
    inchDrive(34, 55, false); //57
    wait(300, msec);
    inchDrive(33, -55, false); //8
    wait(100, msec);

    gyroTurn(80); 
    wait(300, msec);
    inchDrive(18, 55, false);
    gyroTurn(-80);
    wait(300, msec);
    inchDrive(28, 55, false);

    wait(200, msec);
    gyroTurn(-3);
    wait(200, msec);
    inchDrive(58, -55, false);
    wait(200, msec);
    gyroTurn(80); 
    wait(100, msec);
    inchDrive(5, 55, false);
    wait(100, msec);
    gyroTurn(-80); 
    wait(100, msec);
    inchDrive(69, 55, false);
    */
    
    break;  
  
   
    case 2: // SEAN SKILLS 3-12-22 Skills
    inchDrive(60, 40, false); /* og = 40 */ //Push the red alliance goal to the other sidess
    wait(200, msec);
    inchDrive(7, -50, false); /*-50*/ //Back up slightly
    wait(200, msec);
    gyroTurn(-80); //Turn left 90 degrees
    inchDrive(10, 50, false); /*50*/ //Drive to align with the neutral goal
    gyroTurn(-80); //Turn left 90 degrees to face the neutral goal
    inchDrive(30,50, false); /*50*/ //Push the neutral goal back to the homezone

    inchDrive(2,-50, false); /*-50*/ //Back up again
    gyroTurn(80); //Turn right
    inchDrive(25, 50, false); //Align with the middle goal
    gyroTurn(80); //Face the tall goal
    inchDrive(30,  50, false); /*50*/  //Push the tall goal to the other side

    inchDrive(4, -50, false); /*-50*/ //Back up again
    gyroTurn(-80); //Turn left
    inchDrive(25, 50, false); /*50*/ //Align with the second short neutral goal
    gyroTurn(-80); //Face the second short neutral goal
    inchDrive(27, 50, false);/*50*/ //Push the second short neutral goal over

    inchDrive(27, -50, false); //Back up to the blue goal
    backLift.spin(reverse, 100, pct);
    wait(700, msec);
    backLift.stop();
    gyroTurn(-80);
    inchDrive(10, -30, false); //Pick up the blue goal
    backLift.spin(forward);
    inchDrive(30, 100, false); //Drive to the other side of the field
    backLift.stop();
    break;

    case 3: // MIDDLE RUSH
    deploy();
    inchDrive(42, 75, true);
    Claw.set(false);
    Lift.spin(reverse);
    inchDrive(35, -75, false);
    Lift.stop();
    wait(5000, msec);
    break;

    //DO NOTHING AUTON
    case 4:
    deploy();
    wait(15000, msec);
    break;

    //YELLOW RUSH + MIDDLE RUSH

    case 5:

    //testing

    //true = open
    //false= close

    
    //yellow rush
    deploy();
    inchDrive(32, 75, true);
    Claw.set(false);
    wait(300, msec);
    Lift.spin(reverse);
    inchDrive(26, -50, false);
    Lift.stop();
    gyroTurn(45);
    inchDrive(0, 75, true);
    inchDrive(1, 75, true);
    Claw.set(true);
    
    //middle rush
    //inchDrive(2,-75, true);
    wait(200, msec);
    gyroTurn(-86);
    Claw.set(true);
  
    wait(200, msec);
    inchDrive(42, 75, true);
    Claw.set(false);
    Lift.spin(reverse);
    inchDrive(38, -50, false);
    Lift.stop();
   
    break;


    case 6:
    //40 POINT LIFT (EMEGENCY USE)

    deploy();
    

    //lift 
    deploy();
    inchDrive(32, 75, true);
    wait(100, msec);
    Claw.set(false);
    wait(300, msec);
    Lift.spin(reverse);
    
  

    gyroTurn(-75);
    wait(100, msec);
    inchDrive(25, 75, false);
    gyroTurn(70);
    wait(500, msec);

    inchDrive(23 , 75, false);
    wait(400, msec);
    
    lift_up(100);
    wait(1900, msec);

    Lift.setBrake(hold);

    inchDrive(5, 75, false);

    wait(100, msec);

    Claw.set(true);    

    wait(500, msec);

    inchDrive(10, -75, false);


    lift_down(100);
    wait(2000, msec);

    //lift
    /*
    lift_up(75);
    wait(80, msec);
    Lift.stop();
    Lift.setBrake(hold);    
    */
    break;


    case 7:

    //ABBY LIFTING SKILLS

    //lift back lift revserse

    
    //backLift.spin(reverse, 100, pct);   

    deploy();
    backLift.setVelocity(75, pct);
    backLift.spinFor(reverse, 700, degrees, false);
    inchDrive(59, 50, false); //Red Mogo //118
    wait(300, msec); //Short Neutral Goal

    gyroTurn(-58); //left turn
    wait(300, msec);
    inchDrive(13, 55, false); //26

    wait(300, msec);
    gyroTurn(-58);
    wait(300, msec);
  
    inchDrive(15, 55, false); //og 69 
    wait(300, msec);

    //yellow lift 

    Claw.set(false);
    wait(300, msec);
    Lift.spin(reverse);

    inchDrive(16, 55, false); 
    wait(100, msec);

    gyroTurn(58);    
    wait(100, msec);
    inchDrive(15, 75, false);

    wait(100, msec);
    gyroTurn(-58); 

    wait(100,  msec);

    lift_up(100);
    wait(700, msec);

    Lift.setBrake(hold);

    inchDrive(8, 75, false);

    wait(100, msec);

    Claw.set(true); 


    //2nd yellow goal

    wait(100, msec);

    inchDrive(6, -55, false);

    wait(100, msec);

    lift_down(100);
    wait(700, msec);

    gyroTurn(141);

    wait(100, msec);

    inchDrive(15, 55, false);

    //claw grabbing

    Claw.set(false);

    wait(300, msec);

    Lift.spin(reverse);

    wait(700, msec);

    gyroTurn(141);

    wait(300, msec);

    inchDrive(30, 55, false);
    
    lift_up(100);
    wait(700, msec);

    Lift.setBrake(hold);

    inchDrive(3, 75, false);

    wait(100, msec);

    Claw.set(true); 

    //third yellow
     

  
    


   /*

    inchDrive(20, 55, false); //48
    wait(300, msec);
    gyroTurn(-80);
    wait(300, msec);
    inchDrive(34, 55, false); //57
    wait(300, msec);
    inchDrive(33, -55, false); //8
    wait(100, msec);

    gyroTurn(80); 
    wait(300, msec);
    inchDrive(18, 55, false);
    gyroTurn(-80);
    wait(300, msec);
    inchDrive(28, 55, false);

    wait(200, msec);
    gyroTurn(-3);
    wait(200, msec);
    inchDrive(58, -55, false);
    wait(200, msec);
    gyroTurn(80); 
    wait(100, msec);
    inchDrive(5, 55, false);
    wait(100, msec);
    gyroTurn(-80); 
    wait(100, msec);
    inchDrive(69, 55, false);

*/


    break;
  }
}


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
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  
  pre_auton();

  
  while (true) {
    wait(100, msec);
  }
}
