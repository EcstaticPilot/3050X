/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\chiep                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  9-1-21 added pneumatics */
/*                   abby added LEFT auton 9/17/21 */
                  
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
// mogolift             motor         19              
// ClawSpin             motor         7               
// Claw2                digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
// Global Variable
float dia = 4.0;

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

void autonDriver(int wt, int lspeed, int rspeed, int liftspeed, bool claw) {

  LBDrive.spin(forward, lspeed, pct);
  RBDrive.spin(forward, rspeed, pct);
  LFDrive.spin(forward, lspeed, pct);
  RFDrive.spin(forward, rspeed, pct);
  LLift.spin(forward, liftspeed, pct);
  RLift.spin(forward, liftspeed, pct);
  Claw.set(claw);
  wait(wt, msec);
}
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
  Claw.set(claw);
  float c = 0; // our distance
  LBDrive.setRotation(0, degrees);
  Brain.Screen.clearScreen();
  while (fabs(c) <= target) {

    LBDrive.spin(forward, speed, pct);
    RBDrive.spin(forward, speed, pct);
    LFDrive.spin(forward, speed, pct);
    RFDrive.spin(forward, speed, pct);
    Brain.Screen.printAt(1, 40, "inchdrive");
    c = LBDrive.rotation(rev) * 3.14 * dia;
  }
  brakedrive();
  Brain.Screen.clearScreen();
}

void gyroTurn(float target, bool claw) {
  while (Gyro.isCalibrating()) {
    // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    //90 = right, -90 = left
    this_thread::sleep_for(20);
  }
  float heading4 = 0;
  Gyro.setRotation(0, degrees);

  float speed = 0.0;
  float kp = 1.0;
  float d = 2.0;
  Brain.Screen.clearScreen();
  while (fabs(target - heading4) >= d) {
    if (target - heading4 > 0) {
      speed = kp * (target - heading4) + 10;
    }
    if (target - heading4 < 0) {
      speed = kp * (target - heading4) - 10;
    }
    autonDriver(10, speed, -speed, 0, claw);
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

////////////----------------------EOF-----------------------//////////////////////

void pre_auton(void) { wait(2000, msec); }

void autonomous(void) {
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

  // Abby's Code Below
  // auton
  //90 = right, -90 = left
  //90 = right, -90 = left
  // claw true is closed, false is open
  inchDrive(2, 75,  false);
  inchDrive(4, -75, false);
  inchDrive(0, 50, false);
  wait(500, msec);
  inchDrive(118, 75, false);
  wait(500, msec);
  gyroTurn(-90, false);
  wait(400, msec);
  inchDrive(26, 75, false);
  wait(200, msec);
  gyroTurn(-90, false);
  wait(400, msec);
  inchDrive(70, 75, false);
  wait(500, msec); 

  //big one in the middle if yk yk ;)

  inchDrive(10, -75, false);
  wait(200, msec); 
  gyroTurn(90, false);
  wait(400, msec);
  inchDrive(48, 75, false);
  wait(500, msec);
  gyroTurn(90, false);
  wait(400, msec);
  inchDrive(59, 75, false);
  wait(500, msec);
  //inchDrive(15, -75, false);
  wait(500, msec);
  //backing up from big one in middle
  inchDrive(8, -75, false);
  wait(200, msec);

  gyroTurn(-90, false);
  wait(400, msec);
  inchDrive(45, 75, false);
  wait(500, msec);
  gyroTurn(-90, false);
  wait(500, msec);
  inchDrive(65, 75, false);
  wait(500, msec);

  //turning to push yellow
  /*inchDrive(15, -75, false);
  wait(200, msec);
  gyroTurn(90, false);
  wait(100, msec);
  inchDrive(8, 75, false);
  wait(100, msec);
  gyroTurn(-90, false);
  wait(100, msec);
  inchDrive(15, 75, false);
  wait(100, msec);
  gyroTurn(-90, false);
  wait(100, msec);
  inchDrive(20, 75, false);
  wait(100, msec);
  inchDrive(20, -75, false);
  wait(100, msec);
  gyroTurn(90, false);
 */

  //red goal
  inchDrive(118, -75, false);
  wait(200, msec);
  gyroTurn(90, false);
  wait(100, msec);
  inchDrive(28, 75, false);
  wait(200, msec);
  gyroTurn(-90, false);
  wait(200, msec);
  inchDrive(110, 75, false);
  wait(500, msec);

  //going to other side
  //diagonally
  inchDrive(20, -75, false);
  wait(200, msec);
  gyroTurn(45, false);
  wait(100, msec);
  inchDrive(100, -100, false);
  wait(200,msec);
  gyroTurn(45, false);



  //ramp
  /*inchDrive(16, 75, false);
  wait(500, msec);
  balance();

  

  //pushing blue on platform
  gyroTurn(-90, false);
  wait(200, msec);
  inchDrive(8, 75, false);
  balance();

*/

  /*inchDrive(-108, 75, false);
  wait(500, msec);
  gyroTurn(90, false);
  wait(500, msec);
  inchDrive(18, 75, false);
  wait(500, msec);
  gyroTurn(-90, false);
  wait(500, msec);
  inchDrive(108, 75, false);
  wait(500, msec);
*/


  
 //lifting

  /*LLift.spin(forward, 100, pct);
  RLift.spin(forward, 100, pct);
  wait(100, msec);
  LLift.setBrake(coast);
*/
  
  //getting red mogo
  /*Claw.set(false);
  Claw2.set(false);
  wait(100, msec);
  inchDrive(8, -75, false);  
  wait(100, msec);
  gyroTurn(-90, false);
  inchDrive(75, 75, true);

  //lifting

  LLift.spin(forward, 100, pct);
  RLift.spin(forward, 100, pct);
  wait(100, msec);
  LLift.setBrake(coast);
*/
 //pushing last goal
  /*gyroTurn(90, false);
  inchDrive(75, 75, false);
  brakedrive();
*/
  
  
  /*mogolift.spin(reverse, 100, pct);
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
 
  //have to add balance code at the end here
  //has to turn around and get on the balance beam

  //new stuff, need to test out, cuz i didnt use field and might be very cancer
  mogolift.spin(reverse, 100, pct);
  wait(600, msec);
  mogolift.stop();
  //driving a forward a bit to get loose from the mogo
  inchDrive(3, 75, false);
  gyroTurn(-90, false);
  wait(400, msec);

  //going up beam and pusing blue

 
  //balance area
  inchDrive(5, 75, false);
  balance();

 */
 // Felix's Code
 /*float backLiftTime = 1400;
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

 inchDrive(6, 100, false);ax
 gyroTurn(7, false);
 inchDrive(3, 75, false);
 gyroTurn(8, false);
 inchDrive(3, 75, false);
 gyroTurn(50, false);
 inchDrive(100, 75, false);
 inchDrive(60, -75, false);

 //Go for the middle yellow mogo

 gyroTurn(22 , false);
 inchDrive(50, 100, false);
 gyroTurn(-3, 100);
 inchDrive(50, 10, false);

 //Go for the right mogo

 gyroTurn(30,false);
 inchDrive(20, 100, false);
 gyroTurn(15,false);
 inchDrive(100, 100, false);

 gyroTurn(270, 75, -75, false);
 inchDrive(5, 75, false);
 balance();
 */
}

void driverControl() {

  while (true) {
    if (Controller1.ButtonR2.pressing()) {
      Claw.set(true);
    } else if (Controller1.ButtonR1.pressing()) {
      Claw.set(false);
    }
    ClawSpin.setVelocity(100, percent);

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
    } else {
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
      ClawSpin.spin(forward, 20, pct);

    } else if (Controller1.ButtonRight.pressing()) {
      ClawSpin.spin(reverse, 20, pct);

    } else if (Controller1.ButtonA.pressing()) {
      balance();
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
