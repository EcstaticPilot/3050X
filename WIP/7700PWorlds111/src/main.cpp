/*--------------------------------------------------------------------------------------*/
/*                                                                                      */
/*    Module:       main.cpp                                                            */
/*    Author:       student                                                             */
/*    Created:      Fri Apr 15 2022                                                     */
/*    Description:  V5 project                                                          */
/*                                                                                      */
/*    4/15/22: File is Made (Abby)                                                      */
/*                                                                                      */
/*    Changes Made:                                                                     */
/*    4/29 Voltage Drive (Both Match Auton Sides (Work)                                 */
/*                                                                                      */
/*                                                                                      */
/*                                                                                      */
/*                                                                                      */
/*                                                                                      */
/*                                                                                      */
/*                                                                                      */
/*                                                                                      */
/*--------------------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Lift                 motor         1               
// RFDrive              motor         16              
// RMDrive              motor         18              
// RBDrive              motor         19              
// LFDrive              motor         11              
// LMDrive              motor         13              
// LBDrive              motor         12              
// Controller1          controller                    
// Claw                 digital_out   A               
// Backlift             digital_out   B               
// Gyro                 inertial      14              
// ringIntake           motor         2               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

float d = 4.0; //Global Wheel Diameter
float pi = 3.1415926535897932384626;
float g = 7/5;

int autonSelect = 13; //Default // THE CASE
int autonMin = 0;
int autonMax = 20;

//GUI:

  //CASE 0 = YELLOW RUSH
  //CASE 1 = ABBY 100 POINT PUSHING KILLS
  //CASE 2 = 80 POINT SEAN SKILLS
  //CASE 3 = MIDDLE RUSH
  //CASE 4 = DO NOTHING AUTON
  //CASE 5 = YELLOW RUSH + MIDDLE RUSH (DEPOSITING TO LEFT // USED ON LEFT SIDE)
  //CASE 6 = YELLOW RUSH + MIDDLE RUSH (DEPOSITING TO RIGHT // USED ON RIGHT  SIDE)
  //CASE 7 = 40 POINT LIFT (EMERGENCY USE)
  //CASE 8 = ABBY LIFTING SKILLS
  //CASE 9 = HALF AUTON WIN POINT
  //CASE 10: BACK EMERGENCY YELLOW RUSH
  //CASE 11: WIN POINT (YELLOW RUSH +  AND RINGS) ON // RIGHT SIDE
  //CASE 12: WIN POINT (YELLOW RUSH + AND RINGS) ON // LEFT SIDE
  //CASE 13: EVERYTHING RUSH (MIDDLE, YELLOW, WINPOINT + RINGS) // RIGHT SIDE
  //CASE 14: EVERYTHING RUSH (MIDDLE, YELLOW, WINPOINT + RINGS) // LEFT SIDE

//CLAW:

  //TRUE = CLOSE
  //FALSE = OPEN

//GYRO:

  //RIGHT = 90
  //LEFT = -90

//TILE

  //ONE TILE = 24 INCHES X 24 INCHES


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
  Brain.Screen.printAt(20, 40, "LU Temp %f ", LMDrive.temperature(pct));
  Brain.Screen.printAt(20, 50, "RF Temp %f ", RFDrive.temperature(pct));
  Brain.Screen.printAt(20, 60, "RB Temp % f", RBDrive.temperature(pct));
  Brain.Screen.printAt(20, 70 , "RU Temp % f", RMDrive.temperature(pct));
}

void brakeDrive(){
  LFDrive.stop(brake);
  LBDrive.stop(brake);
  LMDrive.stop(brake);
  RFDrive.stop(brake);
  RBDrive.stop(brake);
  RMDrive.stop(brake);
}

void setHold(){
  LFDrive.setBrake(hold);
  LBDrive.setBrake(hold);
  LMDrive.setBrake(hold);
  RFDrive.setBrake(hold);
  RBDrive.setBrake(hold);
  RMDrive.setBrake(hold);
}

void setCoast(){
  LFDrive.setBrake(coast);
  LBDrive.setBrake(coast);
  LMDrive.setBrake(coast);
  RFDrive.setBrake(coast);
  RBDrive.setBrake(coast);
  RMDrive.setBrake(coast);
}

void autonDriver(int wt, int lspeed, int rspeed, int liftspeed, bool claw, int Clawspin, int moggs) 
{
  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct); 
  Lift.spin(forward, liftspeed, pct);
  Claw.set(claw);
  
 //backlift.spin(forward, moggs, pct);
wait(wt, msec);
}

void lift_up(int speed, int wt){
  Lift.spin(forward, speed, pct);
  //Lift.stop();
}

void lift_down(int speed){
  Lift.spin(reverse, speed, pct);
  wait(500, msec);
  Lift.stop();
}

void inchDrive(float target, int speed, bool claw) {
  Claw.set(claw);
  float c = 0; // our distance
  LBDrive.setRotation(0, degrees);
  Brain.Screen.clearScreen();
  while (fabs(c) <= fabs(target)) {

    LBDrive.spin(forward, speed, pct);
    LMDrive.spin(forward, speed, pct);
    RBDrive.spin(forward, speed, pct);
    RMDrive.spin(forward, speed, pct);
    LFDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
    Brain.Screen.printAt(1, 40, "inchdrive");
    c = fabs(LBDrive.rotation(rev) * 3.14 * d);
  }
  brakeDrive();
  Brain.Screen.clearScreen();
}


void Drive(int wt, int lspeed, int rspeed,
           bool driveVolts = false) { // bool = optional var
  if (driveVolts == true) {
    lspeed*=120;
    rspeed*=120;
    LBDrive.spin(forward, lspeed , voltageUnits::mV);
    LFDrive.spin(forward, lspeed , voltageUnits::mV);
    LMDrive.spin(forward, lspeed, voltageUnits::mV);
    RBDrive.spin(forward, rspeed , voltageUnits::mV);
    RFDrive.spin(forward, rspeed , voltageUnits::mV);
    RMDrive.spin(forward, rspeed, voltageUnits::mV);
    

  } else {

    LBDrive.spin(forward, lspeed, pct);
    RBDrive.spin(forward, rspeed, pct);
    LFDrive.spin(forward, lspeed, pct);
    RFDrive.spin(forward, rspeed, pct);
    RMDrive.spin(forward, rspeed, pct);
    LMDrive.spin(forward, lspeed, pct);
  }
  wait(wt, msec);
}

void gyroTurn(float target) {
  while (Gyro.isCalibrating()) {
    // wait for Gyro Calibration , sleep but allow other tasks to run
    //90 = right, -90 = left
    this_thread::sleep_for(20);
  }
  float heading = 0;
  Gyro.setRotation(0, degrees);

  float speed = 0.0;
  float kp = .5; //1
  float d = 1.0;

  //Each float kp == 1.0 if need to change then make it bigger > than 1

  Brain.Screen.clearScreen();
  while (fabs(target - heading) >= d) {
    if (target - heading > 0) {
      speed = kp * (target - heading)+5 ;
    }
    if (target - heading < 0) {
      speed = kp * (target - heading)-5 ;
    }
    //autonDriver(10, speed, -speed, false);
    Drive(10, speed, -speed);
    heading = Gyro.rotation(degrees);
    Brain.Screen.printAt(1, 40, "heading = %.3f", heading);
  }
  brakeDrive();
  //Brain.Screen.clearScreen();
}

void clawOpen(){

  Claw.set(false);
}

void clawClose(){

  Claw.set(true);
}


void backLiftopen(){
  Backlift.set(true);
}

void backLiftclose(){
  Backlift.set(false);
}

//////////////////////////////////////////////////////////////////////////EOF//////////////////////////////////////////////////////////////////////




void autonomous(void) { 

 //CLAW

 //TRUE = OPEN
 //FALSE = CLOSE

 //GYRO

 //RIGHT = 90
 //LEFT = -90

  switch (autonSelect) {

    case 0: //YELLOW RUSH
  
    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    //wait(150, msec);
    inchDrive(38, -100, true);
    wait(200, msec);

    /*
    {
      LFDrive.spin(reverse, 100, pct);
      LMDrive.spin(reverse, 100, pct);
      LBDrive.spin(reverse, 100, pct);
      RFDrive.spin(fwd, 100, pct);
      RMDrive.spin(fwd, 100, pct);
      RBDrive.spin(fwd, 100, pct);
      wait(500,msec);
      LFDrive.stop();
      LMDrive.stop();
      LBDrive.stop();
      RFDrive.stop();
      RMDrive.stop();
      RBDrive.stop();
    }
    wait(200, msec);
    inchDrive(5, -100, false);
    wait(200, msec);
    Lift.spin(reverse, 100, pct);
    {
      LFDrive.spin(fwd, 70, pct);
      LMDrive.spin(fwd, 70, pct);
      LBDrive.spin(fwd, 70, pct);
      RFDrive.spin(reverse, 70, pct);
      RMDrive.spin(reverse, 70, pct);
      RBDrive.spin(reverse, 70, pct);
      wait(385,msec);
      LFDrive.stop();
      LMDrive.stop();
      LBDrive.stop();
      RFDrive.stop();
      RMDrive.stop();
      RBDrive.stop();
    }
    Lift.stop();
    inchDrive(33, 100, false);
    Claw.set(true);
    inchDrive(20, -100, true);

*/

    break;

    case 1: //ABBY SKILLS
    //robot is much faster, changing speed to 50 instead of 75
    //diving inch drive by half
    //balance();


    //deploy();
    //kk
    //backLift.setVelocity(75, pct);
    
    
    inchDrive(59, 75, false); //Red Mogo //118 //59
    wait(500, msec); //Short Neutral Goal


    gyroTurn(-90); //left turn
    wait(500, msec);
    inchDrive(13, 75, false); //26

    wait(300, msec);
    gyroTurn(-90);
    wait(300, msec);
  
    inchDrive(34, 75, false); //og 69 
    wait(300, msec);
    inchDrive(33, -75, false); //46
    wait(300, msec);

    gyroTurn(90); //Tall Neutral Goal
    wait(300, msec);
    inchDrive(20, 75, false); //48
    wait(300, msec);
    gyroTurn(-90);
    wait(300, msec);
    inchDrive(34, 75, false); //57
    wait(300, msec);
    inchDrive(33, -75, false); //8
    wait(100, msec);

    gyroTurn(90); 
    wait(300, msec);
    inchDrive(18, 75, false);
    gyroTurn(-90);
    wait(300, msec);
    inchDrive(28, 75, false);

    wait(200, msec);
    gyroTurn(-3);
    wait(200, msec);
    inchDrive(58, -75, false);
    wait(200, msec);
    gyroTurn(90); 
    wait(100, msec);
    inchDrive(5, 75, false);
    wait(100, msec);
    gyroTurn(-90); 
    wait(100, msec);
    inchDrive(69, 75, false);
    
    break;  
   
    case 2: // SEAN SKILLS 3-12-22 Skills
    inchDrive(60, 40, false); /* og = 40 */ //Push the red alliance goal to the other sidess
    wait(200, msec);
    inchDrive(7, -50, false); /*-50*/ //Back up slightly
    wait(200, msec);
    gyroTurn(-90); //Turn left 90 degrees
    inchDrive(10, 50, false); /*50*/ //Drive to align with the neutral goal
    gyroTurn(-90); //Turn left 90 degrees to face the neutral goal
    inchDrive(30,50, false); /*50*/ //Push the neutral goal back to the homezone

    inchDrive(2,-50, false); /*-50*/ //Back up again
    gyroTurn(90); //Turn right
    inchDrive(25, 50, false); //Align with the middle goal
    gyroTurn(90); //Face the tall goal
    inchDrive(30,  50, false); /*50*/  //Push the tall goal to the other side

    inchDrive(4, -50, false); /*-50*/ //Back up again
    gyroTurn(-90); //Turn left
    inchDrive(25, 50, false); /*50*/ //Align with the second short neutral goal
    gyroTurn(-90); //Face the second short neutral goal
    inchDrive(27, 50, false);/*50*/ //Push the second short neutral goal over

    inchDrive(27, -50, false); //Back up to the blue goal
    //backLift.spin(reverse, 100, pct);
    wait(700, msec);
    //backLift.stop();
    gyroTurn(-90);
    inchDrive(10, -30, false); //Pick up the blue goal
    //backLift.spin(forward);
    inchDrive(30, 100, false); //Drive to the other side of the field
    //backLift.stop();
    break;

    case 3: // MIDDLE RUSH
    inchDrive(42, 75, true);
    Claw.set(false);
    Lift.spin(reverse);
    inchDrive(35, -75, false);
    Lift.stop();
    wait(5000, msec);
    break;

    //DO NOTHING AUTON
    case 4:
    wait(15000, msec);
    break;

    //YELLOW RUSH + MIDDLE RUSH (DEPOSITING TO RIGHT // USED ON RIGHT SIDE)

    case 5:
  
    //true = close
    //false= open
    
    //yellow rush
    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    wait(200, msec);
    //wait(150, msec);
    inchDrive(38, -75, true);
    wait(200, msec);
    gyroTurn(-45);
    wait(400, msec);
    inchDrive(5, 75, false);
    wait(100, msec);
    Claw.set(false);
    wait(200, msec);
    inchDrive(5, -75, false);
    
    
    //middle rush
    //inchDrive(2,-75, true);
    wait(200, msec);
    gyroTurn(83);

    wait(200, msec);

    inchDrive(45, 75, false); //42

    wait(200, msec);

    //true = close
    //false= open

    Claw.set(true);

    wait(300, msec);
   
    inchDrive(38, -75, true);

    break;
    
    case 6:
    //true = close
    //false= open
    //yellow rush rushing through my butthole
    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    wait(200, msec);
    //wait(150, msec);
    inchDrive(38, -75, true);
    wait(200, msec);
    gyroTurn(45);
    wait(400, msec);
    inchDrive(5, 75, false);
    wait(100, msec);
    Claw.set(false);
    wait(200, msec);
    inchDrive(5, -75, false);
    
    //middle rush
    //inchDrive(2,-75, true);
    wait(200, msec);
    gyroTurn(-81);
    wait(200, msec);
    inchDrive(45, 75, false); //42
    wait(200, msec);
    //true = close
    //false= open
    Claw.set(true);
    wait(300, msec);
    inchDrive(38, -75, true);

    break;

    //SOMETHING ELSE
    //j like deex nuts

    case 7:
    //40 POINT LIFT (EMEGENCY USE) (BY SEAN + NO LIFTING)
   
    //lift 
    inchDrive(32, 75, true);
    wait(200, msec);
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
    
    lift_up(100, 100);
    wait(1900, msec);

    Lift.setBrake(hold);

    inchDrive(5, 75, false);

    wait(200, msec);

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

    case 8:

    //ABBY LIFTING SKILLS

    //lift back lift revserse

    //TRUE = CLOSE
    //FALSE = OPEN

    //backLift.spin(reverse, 100, pct);   

    //backLift.setVelocity(75, pct);
    //backLift.spinFor(reverse, 700, degrees, false);

    lift_up(100, 10);
    
    wait(700, msec);

    Lift.setBrake(hold);

    inchDrive(8, 75, true);


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

    Claw.set(true);
    wait(300, msec);
    Lift.spin(reverse);

    inchDrive(16, 55, true); 
    wait(100, msec);

    gyroTurn(58);   
    wait(100, msec);
    inchDrive(15, 75, true);

    wait(100, msec); 
    gyroTurn(-58);

    wait(100,  msec);

    lift_up(100, 100);
    wait(700, msec);

    Lift.setBrake(hold);

    inchDrive(8, 75, true);

    wait(100, msec);

    Claw.set(false); 


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

    Claw.set(true);

    wait(300, msec);

    Lift.spin(reverse);

    wait(700, msec);

    gyroTurn(141);

    wait(300, msec);

    inchDrive(30, 55, true);
    
    lift_up(100, 100);
    wait(700, msec);

    Lift.setBrake(hold);

    inchDrive(3, 75, false);

    wait(100, msec);

    Claw.set(false); 

    //third yellow
    
  
    inchDrive(20, 55, false); //48
    wait(300, msec);
    gyroTurn(-90);
    wait(300, msec);
    inchDrive(34, 55, false); //57
    wait(300, msec);
    inchDrive(33, -55, false); //8
    wait(100, msec);

    gyroTurn(90); 
    wait(300, msec);
    inchDrive(18, 55, false);
    gyroTurn(-90);
    wait(300, msec);
    inchDrive(28, 55, false);

    wait(200, msec);
    gyroTurn(-3);
    wait(200, msec);
    inchDrive(58, -55, false);
    wait(200, msec);
    gyroTurn(90); 
    wait(100, msec);
    inchDrive(5, 55, false);
    wait(100, msec);
    gyroTurn(-90); 
    wait(100, msec);
    inchDrive(69, 55, false);
    
    break;

    case 9:

    //half auton win point
    Claw.set(true);
    wait(100, msec);
    inchDrive(15, 100, false);
    wait(100, msec);
    Claw.set(false);

   break;


    case 10:
   //closes Backlift.set(false);
   inchDrive(33, -75, false);
   Backlift.set(false);
   inchDrive(33, 75, false);
   wait(500, msec);


    break;

    case 11: //win Point
    //yellow rush

    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    wait(200, msec);
    inchDrive(23, -75, true);
    wait(300, msec);
    gyroTurn(-90);
    wait(300, msec);
    Backlift.set(true);
    wait(100, msec);
    inchDrive(15, -75, false);
    wait(300, msec);
    Claw.set(true);
    wait(100, msec);
    Backlift.set(false);
    wait(200, msec);
    
    ringIntake.spin(reverse, 100, pct);
    inchDrive(6, 75, false);

    Backlift.set(true);
    wait(200, msec);
    inchDrive(5, 75, false);

    break;


    case 12:

    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    wait(200, msec);
    inchDrive(23, -75, true);
    wait(300, msec);
    gyroTurn(90);
    wait(300, msec);
    Backlift.set(true);
    wait(100, msec);
    inchDrive(15, -75, false);
    wait(300, msec);
    Claw.set(true);
    wait(100, msec);
    Backlift.set(false);
    wait(200, msec);
    
    ringIntake.spin(reverse, 100, pct);
    inchDrive(6, 75, false);

    Backlift.set(true);
    wait(200, msec);
    inchDrive(5, 75, false);

    break;

    case 13: //RIGHT SIDE

    //GYRO:

   //RIGHT = 90
   //LEFT = -90
    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    wait(200, msec);
    //wait(150, msec);
    inchDrive(38, -75, true);
    wait(200, msec);
    gyroTurn(45);
    wait(400, msec);
    inchDrive(5, 75, false);
    wait(100, msec);
    Claw.set(false);
    wait(200, msec);
    inchDrive(5, -75, false);
    
    //middle rush
    //inchDrive(2,-75, true);
    wait(200, msec);
    gyroTurn(-83);
    wait(200, msec);
    inchDrive(45, 75, false); //42
    wait(200, msec);
    //true = close
    //false= open
    Claw.set(true);
    wait(300, msec);
    inchDrive(22, -75, true);

    wait(500, msec);

    gyroTurn(-50);
    wait(300, msec);
    Backlift.set(true);
    wait(100, msec);
    inchDrive(20, -75, false);
    wait(300, msec);
    Claw.set(true);
    wait(100, msec);
    Backlift.set(false);
    wait(200, msec);
    
    ringIntake.spin(reverse, 100, pct);
    inchDrive(7, 75, false);

    
    break;


    case 14: //LEFT SIDE

    //RIGHT = 90
    //LEFT = -90
    Lift.setBrake(brake);
    inchDrive(33, 100, false);
    //wait(100, msec);
    Claw.set(true);
    wait(200, msec);
    //wait(150, msec);
    inchDrive(38, -75, true);
    wait(200, msec);
    gyroTurn(-45);
    wait(400, msec);
    inchDrive(5, 75, false);
    wait(100, msec);
    Claw.set(false);
    wait(200, msec);
    inchDrive(5, -75, false);
    
    //middle rush
    //inchDrive(2,-75, true);
    wait(200, msec);
    gyroTurn(83);
    wait(200, msec);
    inchDrive(45, 75, false); //42
    wait(200, msec);
    //true = close
    //false= open
    Claw.set(true);
    wait(300, msec);
    inchDrive(22, -75, true);

    wait(500, msec);

    gyroTurn(40);
    wait(300, msec);
    Backlift.set(true);
    wait(100, msec);
    inchDrive(15, -75, false);
    wait(300, msec);
    Claw.set(true);
    wait(100, msec);
    Backlift.set(false);
    wait(200, msec);
    
    ringIntake.spin(reverse, 100, pct);
    inchDrive(7, 75, false);

   break;

  }
}


void usercontrol(void) {

   bool reversed = false;
   bool tiltedUp = false;
   bool aDown = 0; //Variable for when you're trying to reverse
   bool bDown = 0; //Variable for when you're locking the drive
   bool backLifttoggle = false;
   
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

    //Drive Code
    //Voltage Drive
    
    if(!reversed){
      // Drive(10, Controller1.Axis3.position(pct), Controller1.Axis2.position(pct),true);
      
      LBDrive.spin(forward, Controller1.Axis3.position(pct)*120, voltageUnits::mV);
      LFDrive.spin(forward, Controller1.Axis3.position(pct)*120, voltageUnits::mV);
      LMDrive.spin(forward, Controller1.Axis3.position(pct)*120, voltageUnits::mV);
      RBDrive.spin(forward, Controller1.Axis2.position(pct)*120, voltageUnits::mV);
      RFDrive.spin(forward, Controller1.Axis2.position(pct)*120, voltageUnits::mV);
      RMDrive.spin(forward, Controller1.Axis2.position(pct)*120, voltageUnits::mV);
      
    }
    else if(reversed){
      // Drive(10, -(Controller1.Axis2.position(pct)), -(Controller1.Axis3.position(pct)),true);

      LBDrive.spin(reverse, Controller1.Axis2.position(pct)*120, voltageUnits::mV);
      LFDrive.spin(reverse, Controller1.Axis2.position(pct)*120, voltageUnits::mV);
      LMDrive.spin(reverse, Controller1.Axis2.position(pct)*120, voltageUnits::mV);
      RBDrive.spin(reverse, Controller1.Axis3.position(pct)*120, voltageUnits::mV);
      RFDrive.spin(reverse, Controller1.Axis3.position(pct)*120, voltageUnits::mV);
      RMDrive.spin(reverse, Controller1.Axis3.position(pct)*120, voltageUnits::mV);
      
    }
    /*
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
    */
      
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
    if (Controller1.ButtonR2.pressing()){
      Claw.set(true);
    }
    else if (Controller1.ButtonR1.pressing()){
      Claw.set(false);
    }

    //Back Lift
    //up arrow toggles back lift if the back lift is open close it if its closed open it  ll down arrow stops intake
    if (Controller1.ButtonUp.pressing()) // opens
    {
      backLifttoggle = !backLifttoggle;

      Backlift.set(backLifttoggle);

      while (Controller1.ButtonUp.pressing()) {
        wait(10, msec);
      }

    }

    if (Controller1.ButtonDown.pressing()) //ringIntake
    {
      ringIntake.stop();
    }

    //Ring Intake
    //Toggle

    if (Controller1.ButtonLeft.pressing()){
      ringIntake.spin(reverse, 100, pct);
    }

    if (Controller1.ButtonRight.pressing()){
      ringIntake.spin(forward, 100, pct);
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