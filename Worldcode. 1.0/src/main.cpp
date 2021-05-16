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
// Rback                motor         5
// Lback                motor         6
// Arm                  motor         12
// Controller1          controller
// Lintake              motor         16
// Rintake              motor         18
// Lfront               motor         1
// Rfront               motor         10
// Inertial4            inertial      4
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;
// Global Variable
float dia = 4.0;

// tis is GUI : Graphic User Interface
void draw() {

  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(10, 10, 100, 50);
  Brain.Screen.printAt(10, 10, "hi");
  Brain.Screen.printAt(10, 20, "RMotor Temp%f ", Lback.temperature(pct));
  Brain.Screen.printAt(10, 35, "LMotor Temp%f ", Rback.temperature(pct));

  Brain.Screen.setFillColor(blue);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawRectangle(400, 10, 80, 50);
  Brain.Screen.printAt(410, 15, "C++");
}

void motorCurrent() {
  Brain.Screen.printAt(10, 80, "LMotor current%f  ", Lback.current());
  Brain.Screen.printAt(10, 96, "RMotor current%f  ", Rback.current());
  Brain.Screen.printAt(10, 110, "LMotor current%f ", Lfront.current());
  Brain.Screen.printAt(10, 122, "RMotor current%f ", Rfront.current());
  Brain.Screen.printAt(10, 132, "Motor current%f  ", Arm.current());
}

void InchDrive(float target, int speed) {
  int c = 0; // our distance
  Lback.resetRotation();
  while (abs(c) <= target) {
    Lback.spin(forward, speed, pct);
    Rback.spin(forward, speed, pct);
    Lfront.spin(forward, speed, pct);
    Rfront.spin(forward, speed, pct);

    c = Lback.rotation(rev) * 3.14 * dia;
  }

  Lback.stop();
  Rback.stop();
  Lfront.stop();
  Rfront.stop();
}

void Drive(float speed) {
  Lback.spin(forward, speed, pct);
  Rback.spin(forward, speed, pct);
  Lfront.spin(forward, speed, pct);
  Rfront.spin(forward, speed, pct);
}

void Pdrive(float target, float distrav) {
  float error = 0;
  float Dist = 0;
  float kP = 1;
  while (true) {
    Dist = Lfront.position(degrees), (4 * 3.14 / 360);
    error = target - Dist;
    Drive(error * kP);
    wait(5, msec);
  }
}

void autonDriver(int wt, int Lspeed, int Rspeed, int Armspeed, int Lint, int Rint) {
  Lback.spin(forward, Lspeed, pct);
  Rback.spin(forward, Rspeed, pct);
  Lfront.spin(forward, Lspeed, pct);
  Rfront.spin(forward, Rspeed, pct);
  Arm.spin(forward, Armspeed, pct);
  Lintake.spin(forward, Lint, pct);
  Rintake.spin(forward, Rint, pct);
  wait(wt, msec);
}

void gyroTurn(float target, int Lspeed, int Rspeed) {
  while (Inertial4.isCalibrating()) {
    // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    this_thread::sleep_for(20);
  }
  float heading4 = 0;
  Inertial4.setRotation(0, degrees);

  while (fabs(heading4) <= target) {
    heading4 = Inertial4.rotation(degrees);
    autonDriver(10, Lspeed, Rspeed, 0, 0, 0);
    wait(10, msec);
  }

  autonDriver(0, 0, 0, 0, 0, 0);
}

void encPrint() {

  int enc1 = Rfront.velocity(pct);
  int enc2 = Lfront.velocity(pct);
  Brain.Screen.printAt(20, 20, "Right motor front %d ", enc1);
  Brain.Screen.printAt(20, 40, "Left motor front %d ", enc2);
}

void nevergonnagiveyouup()
{ 
  for(int j = 1; j <= 100; j ++){
  Brain.Screen.setPenWidth(9);
  Brain.Screen.printAt(240, 120, "Never");
  wait(500, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Gonna");
  wait(400, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Give");
  wait(800, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "You");
  wait(800, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Up");
  wait(1000, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Never");
  wait(500, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Gonna");
  wait(400, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Let");
  wait(800, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "You");
  wait(800, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Down");
  wait(1000, msec);
  Brain.Screen.clearScreen();
    Brain.Screen.printAt(240, 120, "Never");
  wait(500, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Gonna");
  wait(400, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Run");
  wait(1500, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "Around");
  wait(800, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "And");
  wait(400, msec);
  Brain.Screen.clearScreen();
    Brain.Screen.printAt(240, 120, "Desert");
  wait(1200, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(240, 120, "You.");
  wait(1000, msec);
  Brain.Screen.clearScreen();
  }
}

/*
int rainbowlogo(){
    
    int color1 = 0;
    int a;
    int b;
    int c;
    int d;
    
    while(1==1){
        //Reset values a, b, c, and d
     
        a = b = c = d = 0;
        //Create Random Colors Every Loop
         color1++;
        if(color1 >= 7) color1 = 0;
        if(color1 == 0)Brain.Screen.setPenColor(color::red);   
        else if(color1 == 1) Brain.Screen.setPenColor(color::green); 
        else if(color1 == 2) Brain.Screen.setPenColor(color::blue);     
        else if(color1 == 3) Brain.Screen.setPenColor(color::yellow);  
        else if(color1 == 4) Brain.Screen.setPenColor(color::purple); 
        else if(color1 == 5) Brain.Screen.setPenColor(color::orange);  
        else if(color1 == 6) Brain.Screen.setPenColor(color::cyan); 
     
        //Draw Rainbow Circle
        Brain.Screen.setPenWidth(5);     
        Brain.Screen.drawCircle(240, 120, 105);
         Brain.Screen.setPenWidth(2);
     
        //Draw Rainbow Line
        for(int j = 1; j <= 6; j ++){
            Brain.Screen.drawLine(170 - a, 160, 240 + b, 40 - a);
            a -= 2;
            b++;
        }
     
        Brain.Screen.setPenColor(color::white); 
     
        //Draw 2 White Lines
        for(int k = 1; k <= 6; k ++){
            Brain.Screen.drawLine(250 - c, 60 + d, 310 - d, 160);
            Brain.Screen.drawLine(190 + c, 161 - d, 301, 161 - d);
            d++;
            Brain.Screen.drawLine(250 - c, 60 + d, 310 - d, 160);
            Brain.Screen.drawLine(190 + c, 161 - d, 301, 161 - d);
            c++;
            d++;
        }
        task::sleep(1000); 
     
    }     
}

int rainbowlogo2(){
    
    int color1 = 0;
    int a;
    int b;
    int c;
    int d;
    
    while(1==1){
        //Reset values a, b, c, and d
     
        a = b = c = d = 0;
        //Create Random Colors Every Loop
         color1++;
        if(color1 >= 7) color1 = 0;
        if(color1 == 0)Brain.Screen.setPenColor(color::green);   
        else if(color1 == 1) Brain.Screen.setPenColor(color::red); 
        else if(color1 == 2) Brain.Screen.setPenColor(color::orange);     
        else if(color1 == 3) Brain.Screen.setPenColor(color::purple);  
        else if(color1 == 4) Brain.Screen.setPenColor(color::yellow); 
        else if(color1 == 5) Brain.Screen.setPenColor(color::cyan);  
        else if(color1 == 6) Brain.Screen.setPenColor(color::orange); 
     
        //Draw Rainbow Circle
        Brain.Screen.setPenWidth(5);     
        Brain.Screen.drawCircle(240, 120, 115);
         Brain.Screen.setPenWidth(2);
     
        //Draw Rainbow Line
        for(int j = 1; j <= 6; j ++){
            Brain.Screen.drawLine(170 - a, 160, 240 + b, 40 - a);
            a -= 2;
            b++;
        }
     
        Brain.Screen.setPenColor(color::white); 
     
        //Draw 2 White Lines
        for(int k = 1; k <= 6; k ++){
            Brain.Screen.drawLine(250 - c, 60 + d, 310 - d, 160);
            Brain.Screen.drawLine(190 + c, 161 - d, 301, 161 - d);
            d++;
            Brain.Screen.drawLine(250 - c, 60 + d, 310 - d, 160);
            Brain.Screen.drawLine(190 + c, 161 - d, 301, 161 - d);
            c++;
            d++;
        }
        task::sleep(1000); 
     
    }     
}
*/
//----------------------------------------EOF-------------------------------------------------

void pre_auton(void) {
  while (Inertial4.isCalibrating()) {
    // Wait for Gyro Calibration , Sleep but Allow other tasks to runå
    this_thread::sleep_for(20);
  }
  float heading4 = 0;
  Inertial4.setRotation(0, degrees);
}

void autonomous(void) {
encPrint();
InchDrive(6, 60);
InchDrive(3, -60);
InchDrive(3, 60);


/*
  encPrint();
  InchDrive(3, 100);    // Deploy
  InchDrive(3, -100);   // Deploy
  gyroTurn(9, 80, -80); // turn
  autonDriver(50, 0, 0, 0, 0, 0);
  Lintake.spin(forward, -100, pct);
  Rintake.spin(forward, -100, pct);
  InchDrive(26, 100); // go to middle slightly
  autonDriver(20, -50, -50, 0, 0, 0);
  gyroTurn(43, -80, 80); // turn to goal
  autonDriver(50, 0, 0, 0, 0, 0);
  autonDriver(700, 0, 0, 80, 0, 0); // Lift up intake
  InchDrive(4, 80);                 // forward slightly
  Lintake.stop();
  Rintake.stop();
  autonDriver(800, 0, 0, 20, 80, 80);   // Score
  autonDriver(200, -80, -80, 20, 0, 0); // back slightly

  // Phase two

  gyroTurn(17, 80, -80); // turn to ball
  autonDriver(50, 0, 0, 0, 0, 0);
  autonDriver(800, 0, 0, -80, 0, 0);
  Lintake.spin(forward, -100, pct);
  Rintake.spin(forward, -100, pct);
  InchDrive(32, 80);
  Lintake.stop();
  Rintake.stop();
  autonDriver(50, -10, -10, 0, 0, 0);
  autonDriver(1000, 0, 0, 90, -20, -20);
  gyroTurn(10, -80, 80);
  autonDriver(50, 0, 0, 0, 0, 0);
  */
}

void drivercontrol() {
nevergonnagiveyouup();

  while (true) {
    Brain.Screen.setFillColor(ClrMagenta);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(20, 20, "Left Back Motor Temp %f ", Lback.temperature(pct));
    Brain.Screen.printAt(20, 40, "Right Back Motor Temp %f ", Rback.temperature(pct));
    Brain.Screen.printAt(20, 60, "Left Front Motor Temp %f ", Lfront.temperature(pct));
    Brain.Screen.printAt(20, 80, "Right Front Motor Temp %f ", Rfront.temperature(pct));
    Brain.Screen.printAt(20, 100, "Arm Motor Temp %f", Arm.temperature(pct));
    Brain.Screen.printAt(20, 120, "Right Intake Motor Temp %f", Rintake.temperature(pct));
    Brain.Screen.printAt(20, 140, "Left Intake Motor Temp %f", Lintake.temperature(pct));
    Brain.Screen.setFillColor(white);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(20, 160, "Never gonna give you up,");
    Brain.Screen.printAt(20, 180, "Never gonna let you down,");
    
    Lback.spin(forward, Controller1.Axis3.position(pct), pct);
    Rback.spin(forward, Controller1.Axis2.position(pct), pct);
    Lfront.spin(forward, Controller1.Axis3.position(pct), pct);
    Rfront.spin(forward, Controller1.Axis2.position(pct), pct);

    if (Controller1.ButtonR1.pressing()) {
      Lintake.spin(forward, 100, pct);
      Rintake.spin(forward, 100, pct);

    } else if (Controller1.ButtonR2.pressing()) {
      Lintake.spin(reverse, 100, pct);
      Rintake.spin(reverse, 100, pct);

    }

    else {
      Lintake.stop(brake);
      Rintake.stop(brake);
    }
    if (Controller1.ButtonL1.pressing()) {
      Arm.spin(forward, 85, pct);

    } else if (Controller1.ButtonL2.pressing()) {
      Arm.spin(reverse, 85, pct);
    }

    else {
      Arm.stop(brake);
    }
  }
}

int main() {
  vexcodeInit();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  pre_auton();
  // autonomous ();
  // driverControl();
  while (true) {
    wait(100, msec);
  }
  // Run the pre-autonomous function.
}
// Initializing Robot Configuration. DO NOT REMOVE!
