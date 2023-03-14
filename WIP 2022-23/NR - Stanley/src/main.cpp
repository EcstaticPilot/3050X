
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// F2                   motor         15              
// Injector             digital_out   A               
// LF                   motor         18              
// LB                   motor         12              
// RF                   motor         20              
// RB                   motor         4               
// Intake1              motor         1               
// turret               motor         9               
// gyro1                inertial      11              
// RotationL            rotation      5               
// RotationB            rotation      3               
// turretG              inertial      14              
// Color                optical       7               
// TurretE              rotation      17              
// turretOptical        optical       2               
// roller               motor         6               
// expansion            digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
using namespace vex;

competition Competition;
extern float X;
extern float Y;
// drive.cpp
void drive(int lSpeed, int rSpeed, double wt);
void drive_brake(vex::brakeType Brake=brake);
void rotate(double dir, double accuracy = 1);
void inchDrive(double target, double speedMod=1);
void forward_dist(float dist);
void DriveToPoint(double targetX, double targetY, float speedMult = 1);
void RAMSETE(float targetX, float targetY, float targetAngle,float accuracy=1);
void DriveToPoint2(float targetX,float targetY);
//odometry.cpp
int odometery();



int ControllerPrint() {

  Brain.Timer.reset();
 
  while (1) {

    Controller2.Screen.setCursor(2, 1);
    Controller2.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller2.Screen.setCursor(3, 1);
   
    this_thread::sleep_for(75);
  }
}

void pre_auton(void) {

  vexcodeInit();

  gyro1.calibrate();

  waitUntil(!(gyro1.isCalibrating()));
    thread ControllerPrinting = thread(ControllerPrint);

}

void autonomous(void) {}

void usercontrol(void) {

  while (true) {
   
    wait(10, msec);
  }
}

int main() {
  Competition.bStopAllTasksBetweenModes=false;
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

   while (true) {
     wait(100, msec);
   }
}