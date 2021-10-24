/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      Sat Jul 17 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftBack             motor         11              
// LeftFront            motor         1               
// RightBack            motor         20              
// RightFront           motor         10              
// Gyro                 inertial      17              
// RL19                 motor         19              
// LL12                 motor         12              
// Grabber              digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----`


#include "vex.h"
competition Competition;
using namespace vex;
float pi = 3.14;
float Diamter = 4.0;
float scale = 12000/100;

void drive(int lspeed, int rspeed,  int dtime){
  LeftBack.spin(forward, lspeed, pct);
  LeftFront.spin(forward, lspeed, pct);
  RightFront.spin(forward, rspeed, pct);
  RightFront.spin(forward, rspeed, pct);
  wait(dtime, msec); 
}

void driveVoltage(double lspeed, double rspeed, double dtime)
{
  LeftBack.spin(forward, lspeed*scale, voltageUnits::mV);
  LeftBack.spin(forward, lspeed*scale, voltageUnits::mV);
  RightBack.spin(forward, rspeed*scale, voltageUnits::mV);
  RightFront.spin(forward, rspeed*scale, voltageUnits::mV);
  wait(dtime, sec);
}

void grab(bool close = true)
{
  Grabber.set(close);
}

void pre_auton()
{
  wait(500, msec);
}

void auton()
{
 
}

void inchDrive(float target, int speed){
  LeftFront.setPosition(0, rev);
  float inches = 0.0;
  float turns=0.0;
  while (fabs(inches) < fabs(target)) {
    drive(speed, speed ,10);
    turns = LeftFront.position(rev);
    inches = turns * Diamter * pi;
    Brain.Screen.printAt(1,100, "turns = %.2f inches=%.2f", turns, inches);
  }
  drive(0,0,0);
}
//need to add a gyro
//PID TURN D = DERIVITIVE, SLOPE OF A FUNCTION
void gyroturn(float target){ //turning gyro 
  float kp=2.0;
  float kd = 16.0;
  Gyro.setRotation(0, degrees); //setting gyro to 0
  float heading = 0.0;
  float speed = 100;
  float error = target;
  float olderror=error;
  while(fabs(error)>2.0){ //fabs = absolute value
    heading= Gyro.rotation(degrees);
    olderror=error;
    error = target-heading; //the error gets smaller when u reach ur target
    drive(speed, -speed, 10);
    speed = kp*error+kd*(error-olderror); //when error is big go fast when error is small go slow, find new error
    Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", heading); // formula for 2 decimal places
    Brain.Screen.printAt(1, 60, "speed = %0.2f    degrees", speed);
  }
  drive(0,0,0);
  heading= Gyro.rotation(degrees);
  Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", heading);
}



void driver(){
  
  Brain.Screen.printAt(1,  40,  "Hello  Friends Driver is Running");
  // User control code here, inside the loop
   LL12.setStopping(brake);
    RL19.setStopping(brake);
  while (true){
    double lstick=Controller1.Axis3.position();
    double rstick=Controller1.Axis2.position();
    drive(lstick,rstick, 10);
    driveVoltage(lstick, rstick, 10);
    if(Controller1.ButtonR1.pressing())
    {
      grab();
    }
    if(Controller1.ButtonR2.pressing())
    {
      grab(false);
    }

    if(Controller1.ButtonL1.pressing()){
      LL12.spin(forward, 75, pct);
      RL19.spin(forward, 75, pct);
  }
    else if(Controller1.ButtonL2.pressing()){
      LL12.spin(reverse, 75, pct);
      RL19.spin(reverse, 75, pct);

  }
    else{
      LL12.stop();
      RL19.stop();
  }
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

 


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);
  
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);

  }

}
