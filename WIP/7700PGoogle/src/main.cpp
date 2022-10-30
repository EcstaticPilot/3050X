/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\chiep                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  9-1-21 added pneumatics */
/*                   abby added LEFT auton 9/17/21 */
//after skills comp on 1/29/22 everything fixed and updated
//without neumatics
                  
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RBDrive              motor         20              
// LBDrive              motor         11              
// LLift                motor         2               
// Controller1          controller                    
// LFDrive              motor         15              
// Gyro                 inertial      13              
// RLift                motor         9               
// Claw                 digital_out   C               
// RFDrive              motor         10              
// mogolift             motor         19              
// ClawSpin             motor         16              
// Claw2                digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
// Global Variable
float dia = 4.0;
int autonSelect = 2;
int autonMin = 0;
int autonMax = 3;
// tis is GUI : Graphic User Interface

void Drive(int wt, int lspeed, int rspeed,
           bool driveVolts = false) { // bool = optional var
  if (driveVolts == true) {
    lspeed*=120; // * = multiply
    lspeed*=120;
    LBDrive.spin(forward, lspeed , voltageUnits::mV);
    LFDrive.spin(forward, lspeed , voltageUnits::mV);
    RBDrive.spin(forward, rspeed , voltageUnits::mV);
    RFDrive.spin(forward, rspeed , voltageUnits::mV);

  } else {

    LBDrive.spin(forward, lspeed, pct);
    RBDrive.spin(forward, rspeed, pct);
    LFDrive.spin(forward, lspeed, pct);
    RFDrive.spin(forward, rspeed, pct);
  }
  wait(wt, msec);
}

void claw(bool claw) { Claw.set(claw); }

void lift(int liftspeed) {
  LLift.spin(forward, liftspeed, pct);
  RLift.spin(forward, liftspeed, pct);
}
void draw() {

  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(10, 10, 100, 50);
  Brain.Screen.printAt(10, 10, "hi");
  Brain.Screen.printAt(10, 20, "RMotor Temp%f ", LBDrive.temperature(pct));
  Brain.Screen.printAt(10, 35, "LMotor Temp%f ", RBDrive.temperature(pct));
  // Brain.Screen.printAt(10, 40,"Motor");

  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(400, 10, 80, 50);
  Brain.Screen.printAt(410, 15, "C++");
}

void motorCurrent() {
  /* Brain.Screen.printAt( 10, 80,"LMotor current%f  ",LeftBackMotor.current());
   Brain.Screen.printAt( 10, 96,"RMotor current%f  ",RightBackMotor.current());
   Brain.Screen.printAt( 10, 110,"LMotor current%f ",Lfront.current());
   Brain.Screen.printAt( 10, 122,"RMotor current%f ",Rfront10.current());
   Brain.Screen.printAt( 10, 132,"Motor current%f  ",Arm.current());
   */
}
void autonDriver1(int wt, int lspeed, int rspeed, int liftspeed, bool claw) 
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
void autonDriver2(int wt, int lspeed, int rspeed, int liftspeed, bool claw, int Clawspin, int moggs) 
{

  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct); 
  LLift.spin(forward, liftspeed, pct);
  RLift.spin(forward, liftspeed, pct);
  Claw.set(claw);
  ClawSpin.spin(forward, Clawspin, pct);
  mogolift.spin(forward, moggs, pct);
wait(wt, msec);
}

<<<<<<< Updated upstream


=======
>>>>>>> Stashed changes
void brakedrive() {
  RBDrive.stop(brake);
  LBDrive.stop(brake);
  RFDrive.stop(brake);
  LFDrive.stop(brake);
}
void coastdrive(){
  RBDrive.stop(coast);
  LBDrive.stop(coast);
  RFDrive.stop(coast);
  LFDrive.stop(coast);
}

void inchDrive(float target, int speed, bool claw) {
  Claw2.set(claw);
  Claw.set(claw);
  float c = 0; // our distance
  LBDrive.setRotation(0, degrees);
  Brain.Screen.clearScreen();
  while (fabs(c) <= fabs(target)) {

    LBDrive.spin(forward, speed, pct);
    RBDrive.spin(forward, speed, pct);
    LFDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
    Brain.Screen.printAt(1, 40, "inchdrive");
    c = fabs(LBDrive.rotation(rev) * 3.14 * dia);
  }
  brakedrive();
  Brain.Screen.clearScreen();
}
void gyroTurn1(float target, int Lspeed, int Rspeed, bool claw)
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
    autonDriver1(10 , speed, -speed, 0, claw);
    
    wait(10,msec);
    heading4=Gyro.rotation(degrees); 
  }
  autonDriver1(0, 0, 0, 0, true);
}
void gyroTurn2(float target, bool claw, float &facing) {
  Claw2.set(claw);
  Claw.set(claw);
  while (Gyro.isCalibrating()) {
    // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    //90 = right, -90 = left
    this_thread::sleep_for(20);
  }
  float heading4 = 0;
  //Gyro.setRotation(0, degrees);

  float speed = 0.0;
  float kp = 1.0;
  float d = 2.0;
  facing += target;
  target=facing;

  Brain.Screen.clearScreen();
  while (fabs(target - heading4) >= d) {
    if (target - heading4 > 0) {
      speed = kp * (target - heading4) + 10;
    }
    if (target - heading4 < 0) {
      speed = kp * (target - heading4) - 10;
    }
    
    autonDriver2(10, speed, -speed, 0, claw, 0,0);
    //Drive(10, speed, -speed);
    heading4 = Gyro.rotation(degrees);
    Brain.Screen.printAt(1, 40, "heading = %.3f", heading4);
  }
  brakedrive();
  //Brain.Screen.clearScreen();
}



void balance() {
  float pitch = Gyro.pitch(deg);
  float oldpitch = pitch;
  inchDrive(10, 75, false);
  Brain.Screen.clearScreen();
  float kp = 1;
  float kd = 10.0; //20
 
  // float d=0.5;
  while (true) //(fabs(pitch)>d)
  {
    float speed = kp * pitch + kd * (pitch - oldpitch);
    Drive(10, speed, speed);
    oldpitch = pitch;
    pitch = Gyro.pitch(deg);
    Brain.Screen.printAt(1, 100, "pitch=   %.3f   ", pitch);
  }
  brakedrive();
}

void drawGUI() {
  // Draws 2 buttons to be used for selecting auto
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
   
    if(autonSelect==0){
      Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           Yellow Rush");
    }
    else if(autonSelect==1){
      Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           Sarah Skills");
    }
    else if(autonSelect==2){
      Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           Middle Rush");
    }
    else if(autonSelect==3){
      Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           Abby Skills");
    }
    else{
      Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           None Selected");
    }
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

////////////----------------------EOF-----------------------//////////////////////

void pre_auton(void) { 
 // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.printAt(1, 40, "pre auton is running");
  drawGUI();
  Brain.Screen.pressed(selectAuton);
}

void autonomous(void) {
  float facing=0;
  while (Gyro.isCalibrating()) {

    wait(20, msec);
  }

  // autonDriver(wait time, left drive speed, right drive speed, lift speed,
  // claw close is true)
  Brain.Screen.printAt(20, 20, "Motor Temp%f ", LBDrive.temperature(pct));
  Brain.Screen.printAt(20, 40, "Motor Temp%f ", RBDrive.temperature(pct));
  Brain.Screen.printAt(20, 60, "Motor Temp%f ", LFDrive.temperature(pct));
  Brain.Screen.printAt(20, 80, "Motor Temp%f ", RFDrive.temperature(pct));
  Brain.Screen.printAt(20, 100, "Heading%f ", Gyro.rotation(deg));
  // claw true is closed, false is open
  // happy face
  Brain.Screen.setFillColor(yellow);
  Brain.Screen.drawCircle(360, 125, 110);
  Brain.Screen.setPenColor(black);
  Brain.Screen.drawLine(315, 180, 400, 180);
  Brain.Screen.drawLine(315, 181, 400, 181);
  Brain.Screen.drawLine(315, 182, 400, 182);
  Brain.Screen.drawLine(315, 183, 400, 183);
  // end of first line
  Brain.Screen.drawLine(290, 100, 350, 100);
  Brain.Screen.drawLine(290, 101, 350, 101);
  Brain.Screen.drawLine(290, 102, 350, 102);
  Brain.Screen.drawLine(290, 103, 350, 103);
  // end of second line
  Brain.Screen.drawLine(375, 100, 430, 100);
  Brain.Screen.drawLine(375, 101, 430, 101);
  Brain.Screen.drawLine(375, 102, 430, 102);
  Brain.Screen.drawLine(375, 103, 430, 103);
  // printing some words :)
  Brain.Screen.setFillColor(green);
/*
  Brain.Screen.printAt(50, 150, "PPPPPPPPPPPPPPPPPPPPPPPPPP");
  Brain.Screen.printAt(50, 150, "PPPP                    PPPP");
  Brain.Screen.printAt(50, 149, "PPPP                      PPPP");
  Brain.Screen.printAt(50, 150, "PPPP                         PPPP");
  Brain.Screen.printAt(50, 151, "PPPP                          PPPP");
  Brain.Screen.printAt(50, 152, "PPPP                           PPPP");
  Brain.Screen.printAt(50, 153, "PPPP                           PPPP");
  Brain.Screen.printAt(50, 154, "PPPP                           PPPP");
  Brain.Screen.printAt(50, 155, "PPPP                           PPPP");
  Brain.Screen.printAt(50, 156, "PPPP                          PPPP");
  Brain.Screen.printAt(50, 153, "PPPP                          PPPP");
  Brain.Screen.printAt(50, 153, "PPPP                         PPPP");
  Brain.Screen.printAt(50, 153, "PPPP                       PPPP");
  Brain.Screen.printAt(50, 153, "PPPP                     PPPP");
  Brain.Screen.printAt(50, 153, "PPPP                   PPPP");
  Brain.Screen.printAt(50, 153, "PPPPPPPPPPPPPPPPPPPPPPPP");
  Brain.Screen.printAt(50, 153, "PPPP");
  Brain.Screen.printAt(50, 154, "PPPP");
  Brain.Screen.printAt(50, 155, "PPPP");
  Brain.Screen.printAt(50, 156, "PPPP");
  Brain.Screen.printAt(50, 157, "PPPP");
  Brain.Screen.printAt(50, 158, "PPPP");
  Brain.Screen.printAt(50, 159, "PPPP");
  Brain.Screen.printAt(50, 160, "PPPP");
  Brain.Screen.printAt(50, 161, "PPPP");
  Brain.Screen.printAt(50, 162, "PPPP");
  Brain.Screen.printAt(50, 163, "PPPP");
*/
switch(autonSelect)
{

  case 0: //Yellow Rush

autonDriver2(500, 0, 0, 0, false, -70, 0);
autonDriver2(600, 0, 0, 70, false, 0, 0);
autonDriver2(500, 0, 0, 0, false, 80, 0);
autonDriver2(600, 0, 0, -70, true, 0, 0);
LLift.setBrake(hold);
RLift.setBrake(hold);

inchDrive(65, 100, true);
autonDriver1(10, 0, 0, 0, false);
wait(300, msec);
inchDrive(65, -100, false);

 wait(15000, msec);

  break;
   case 1: 
   //Deploy
<<<<<<< Updated upstream
autonDriver(500, 0, 0, 0, false, -70, 0);
autonDriver(700, 0, 0, 70, false, 0, 0);
autonDriver(500, 0, 0, 0, false, 80, 0);
autonDriver(700, 0, 0, -70, true, 0, 0);

mogolift.spin(reverse, 100, pct);
wait(1500, msec);
mogolift.stop(brake);
inchDrive(18, 100, false);
mogolift.spin(forward, 100, pct);
wait(1100, msec);
mogolift.stop(brake);
gyroTurn(150, false, facing);

=======
//false = closed, true = open
autonDriver2(500, 0, 0, 0, false, -70, 0);
autonDriver2(600, 0, 0, 70, false, 0, 0);
autonDriver2(500, 0, 0, 0, false, 80, 0);
autonDriver2(600, 0, 0, -70, false, 0, 0);//eekeeekeee

//Blue left back lift
autonDriver2(2000, 0, 0, 0, false, 0, -70);
inchDrive(4, -60, false);
autonDriver2(1900, 0, 0, 0, false, 0, 70);
mogolift.setBrake(hold);

//left neutral mogo

gyroTurn2(20, false, facing);
inchDrive(5, 60, false);
gyroTurn2(72, false, facing);
inchDrive(60, 100, true);
inchDrive(50, 100, false); 
gyroTurn2(80, false, facing);

//Put on tray
autonDriver2(2000, 0, 0, 100, false, 0, 0);
LLift.setBrake(hold);
RLift.setBrake(hold);
inchDrive(37, 60, false);
gyroTurn2(-75, false, facing);
autonDriver2(800, 100, 100, 0, false, 0, 0);
autonDriver2(300, 0, 0, -40, false, 0, 0);
autonDriver2(10, 0, 0, 0, true, 0, 0);
inchDrive(10, -60, true);
>>>>>>> Stashed changes


  break;
   case 2: //Middle Mogo
autonDriver2(500, 0, 0, 0, false, -70, 0);
autonDriver2(600, 0, 0, 70, false, 0, 0);
autonDriver2(450, 0, 0, 0, false, 80, 0);
autonDriver2(600, 0, 0, -70, true, 0, 0);
LLift.setBrake(hold);
RLift.setBrake(hold);

inchDrive(80, 100, true);
autonDriver1(10, 0, 0, 0, false);
wait(300, msec);
inchDrive(80, -100, false);

/*
gyroTurn1(10, -60, 60, false);
autonDriver1(10, 0, 0, 0, true);
inchDrive(3, -60, true);
gyroTurn1(45, 60, -60, true);
inchDrive(80, 100, true);
autonDriver1(10, 0, 0, 0, false);
wait(300, msec);
inchDrive(80, -100, false);
*/
 wait(15000, msec);

  break;
   case 3:

// Abby's Code Below
  // auton
  //90 = right, -90 = left
  //90 = right, -90 = left
 
 // claw true is open, false is closed
 //Claw.set(true);
 // Claw2.set(true);
  inchDrive(2, 75,  false);
  inchDrive(0, 50, false);

  inchDrive(118, 75, false);
  wait(500, msec);
  gyroTurn1(-90, 75, 75, false);
  //gyroTurn(-90, false,facing);
  wait(400, msec);
  inchDrive(26, 75, false);

  wait(200, msec);
  gyroTurn1(-90, 75, 75, false);
  //gyroTurn(-90, false, facing);
  wait(400, msec);
  
  inchDrive(69, 75, false);
  wait(500, msec);
  inchDrive(6, -75, false);
  wait(500, msec);
  
//really frikin big one in the middle if yk yk ;)
 gyroTurn1(90, 75, 75, false);
  //gyroTurn(90, false ,facing);
  wait(400, msec);
  inchDrive(48, 75, false);
  wait(500, msec);
  gyroTurn1(90, 75, 75, false);
  //gyroTurn(90, false ,facing);
  wait(500, msec);
  inchDrive(57, 75, false);
  wait(500, msec);
  inchDrive(8, -75, false);
  wait(100, msec);

  //third yellow
  gyroTurn1(-90, 75, 75, false);
 // gyroTurn(-90, false ,facing);
  wait(100, msec);
  inchDrive(50, 75, false);
  wait(100, msec);
  gyroTurn1(-90, 75, 75, false);
  //gyroTurn(-90, false ,facing);
  wait(100, msec);
  inchDrive(70, 75, false);
  wait(500, msec);

  //red goal

  inchDrive(108, -75, false);
  wait(500, msec);
  gyroTurn1(90, 75, 75, false);
  //gyroTurn(90, false ,facing);
  wait(500, msec);
  inchDrive(25, 75, false);
  wait(500, msec);
  gyroTurn1(-90, 75, 75, false);
  //gyroTurn(-90, false ,facing);
  wait(500, msec);
  inchDrive(108, 75, false);
  wait(500, msec);
  break;
}
  
}

void driverControl() {

  while (true) {
    if (Controller1.ButtonR2.pressing()) {
      Claw.set(false);
      //Claw2.set(false);

    } else if (Controller1.ButtonR1.pressing()) {
      Claw.set(true);
      //Claw2.set(true);

    }
    //ClawSpin.setVelocity(100, percent);

    Brain.Screen.printAt(160, 60, "hi");
    Brain.Screen.printAt(20, 20, "LB Motor Temp%f ", LBDrive.temperature(pct));
    Brain.Screen.printAt(20, 40, "RB Motor Temp%f ", RBDrive.temperature(pct));
    Brain.Screen.printAt(20, 60, "LF Motor Temp%f ", LFDrive.temperature(pct));
    Brain.Screen.printAt(20, 80, "RF Motor Temp%f ", RFDrive.temperature(pct));

    LBDrive.spin(forward, Controller1.Axis3.position(pct), pct);
    LFDrive.spin(forward, Controller1.Axis3.position(pct), pct);
    RFDrive.spin(forward, Controller1.Axis2.position(pct), pct);
    RBDrive.spin(forward, Controller1.Axis2.position(pct), pct);

    if (Controller1.ButtonX.pressing()) {
      LFDrive.setBrake(hold);
      LBDrive.setBrake(hold);
      RFDrive.setBrake(hold);
      RBDrive.setBrake(hold);
    } else if (Controller1.ButtonY.pressing()) {
      LFDrive.setBrake(coast);
      LBDrive.setBrake(coast);
      RFDrive.setBrake(coast);
      RBDrive.setBrake(coast);
    }

    if (Controller1.ButtonL1.pressing()) {
      LLift.spin(forward, 100, pct);
      RLift.spin(forward, 100, pct);

    } else if (Controller1.ButtonL2.pressing()) {
      LLift.spin(reverse, 100, pct);
      RLift.spin(reverse, 100, pct);
    }

    else {
      LLift.stop(brake);
      RLift.stop(brake);
    }

    if (Controller1.ButtonLeft.pressing()) {
      ClawSpin.spin(forward, 70, pct);

    } else if (Controller1.ButtonRight.pressing()) {
      ClawSpin.spin(forward, -70, pct);

    } else {
      ClawSpin.stop(hold);
    }

    if (Controller1.ButtonUp.pressing()) {
      mogolift.spin(forward, 100, pct);
    } else if (Controller1.ButtonDown.pressing()) {
      mogolift.spin(reverse, 100, pct);
    } else {
      mogolift.stop(hold);
    }
  }
}

int main() {
  vexcodeInit();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(driverControl);

  pre_auton();

  // autonomous ();
  // driverControl();
  while (true) {
    wait(100, msec);
  }
  // Run the pre-autonomous function.
}
// Initializing Robot Configuration. DO NOT REMOVE!
