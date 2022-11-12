
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       NR 7700P                                                  */
/*    Created:      May 13, 2022                                              */
/*    Description:  code of Nikhil Ramanuja 7700P turret bot vex spin up      */
/*----------------------------------------------------------------------------*/
//vex::vision::signature BGOAL = vex::vision::signature (1, -1855, 2553, 349, 2243, 10039, 6141, 1, 0);
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// F1                   motor         2               
// F2                   motor         15              
// Injector             digital_out   A               
// LF                   motor         21              
// LB                   motor         12              
// RF                   motor         20              
// RB                   motor         4               
// Intake1              motor         1               
// turret               motor         19              
// gyro1                inertial      11              
// RotationL            rotation      6               
// RotationB            rotation      3               
// turretG              inertial      14              
// Color                optical       7               
// TurretE              rotation      17              
// Vision16             vision        16              
// turretOptical        optical       9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
//#include "sylib.hpp"
double GoalAngle = 0;
double TargetSpeed = 0.0;
using namespace vex;
// 100 digits of pi because I like Pi𝝿
long double pi = 3.14159265358979323;
// A global instance of competition
competition Competition;

void vision_sensor() {

float error = 0.0;
float accuracy = 10;

Vision16.takeSnapshot(Vision16__SIG_1);

if (Vision16.largestObject.exists == true) {
  double goal_position = Vision16.largestObject.centerX;
// 158 is the x-coordinate for the center of the vision sensor
error = goal_position - 158;

  while (fabs(error) < accuracy) {
    
  }
}
}

void flywheelMonitor();
void spinFlywheel(double);
// define your global instances of motors and other devices here
double OldError = 0.0;
double TBHval = 0.0;
double FWDrive = 0.0;
bool TurretToggle = false;
void drive (int lSpeed,int rSpeed,double wt){
    LF.spin(forward, lSpeed, pct);
    RF.spin(forward, rSpeed, pct);
    LB.spin(forward, lSpeed, pct);
    RB.spin(forward, rSpeed, pct);
    wait(wt, msec);
  LF.stop();
    RF.stop();
    LB.stop();
    RB.stop();
}
void inchDrive(double dist, double speedMod = 1,double
  stopTime=99999999999999999, double accuracy = 0.5) { 
   double startPos =RotationL.position(deg);
  dist = -dist;
  
  double currDist = 0;
  double speed;
  double error = dist;
  double prevError = error;
  double Kp = 16.667;   // Porportional
  double Ki = 0.5;      // Integral
  double Kd = 13.33336; // Derivative
  double sum = 0;

  while ((fabs(error) > accuracy ||
          fabs(speed) > 10)) {
 
  // && !(wallStop && errors[0] == errors[1] &&
                               // errors[1] == errors[2] && errors[2] ==
                              //  errors[3] && errors[0] != dist)) {
    currDist = (RotationL.position(deg)-startPos)*(pi/180)*(2.75/2);
    error = dist - currDist;
    sum = sum * 0.8 + error;
    speed = Kp * error + Ki * sum + Kd * (error - prevError);

    drive(speed * speedMod, speed * speedMod, 10);

    prevError = error;
    
  }
               }

void controlFlywheel1(double target) {
  double speed = F1.velocity(pct);

  spinFlywheel((target - speed) + target);
 // Brain.Screen.printAt(180, 40, "fwdrive %.1f  ", target);

 // Brain.Screen.printAt(1, 40, " speed = %.2f ", speed);
}
// ODOMETERY

double X = 0, Y = 0; // declare global x and y
double prevHeading = gyro1.heading();
double deltaHeading = 0; // change in heading
double absoluteOrientation = pi;
double localX;            // local x to use in loop
double localY;            // local y to use in loop
double lRad = 1.375;      // radius of tracking wheel
double bRad = 1.375;      // radius of tracking wheel
double Sl = 3.75;         // distance of left wheel to tracking center
double Sb = 2.6;          // distance of back wheel to tracking center
double lEncoder = 0;      // declaring encoder variable left
double bEncoder = 0;      // declaring encoder variable back
double distL = 0;         // distance left encoder has traveled
double distB = 0;         // distance back encoder has traveled
double prevLE = lEncoder; // create previous encoder value left
double prevBE = bEncoder; // create previous encoder value back
double averageHeading;    // 
double deltaX =
        (localY * cos(averageHeading)) - (localX * sin(averageHeading));
    double deltaY =
        (localX * cos(averageHeading)) - (localY * sin(averageHeading));
int odometery() {
   
  Controller1.rumble(".");
  RotationL.resetPosition();
  RotationB.resetPosition();
  while (1) {
    Brain.Screen.printAt(1, 20, " prevHeading = %.2f ", prevHeading);
    Brain.Screen.printAt(1, 40, " deltaHeading = %.2f ", deltaHeading);
    Brain.Screen.printAt(1, 60, " absoluteOrientation = %.2f ", absoluteOrientation);
    Brain.Screen.printAt(1, 80, " localX = %.2f localY = %.2f ", localX, localY);
    Brain.Screen.printAt(1, 100, " lEncoder = %.2f bEncoder = %.2f", lEncoder, bEncoder);
    Brain.Screen.printAt(1, 120, " distL = %.2f distB = %.2f ", distL,distB);
    Brain.Screen.printAt(1, 140, " deltaX = %.2f deltaY = %.2f", deltaX, deltaY);
   Brain.Screen.printAt(1, 160, " averageHeading = %.2f ", averageHeading);
   

    lEncoder = RotationL.position(degrees);
    bEncoder = RotationB.position(degrees);

    distL = ((lEncoder - prevLE) * pi / 180) * lRad;
    // convert encoder distance into distance traveled
    distB = ((bEncoder - prevBE) * pi / 180) * bRad;
    // convert encoder distance into disntance traveled

    prevLE = lEncoder; // create previous encoder value left
    prevBE = bEncoder; // create previous encoder value back

    absoluteOrientation = gyro1.heading(degrees) * pi / 180.0;
    deltaHeading =
        absoluteOrientation - prevHeading; // calculate change in heading
averageHeading = prevHeading+(deltaHeading)/2;
    prevHeading = absoluteOrientation;
    
    deltaX =
        (distL * sin(averageHeading)) + (distB * cos(averageHeading));
    deltaY =
        (distL * cos(averageHeading)) - (distB * sin(averageHeading));
    while (absoluteOrientation >= 2 * M_PI) {
      absoluteOrientation -= 2 * M_PI;
    }

    while (absoluteOrientation < 0) {
      absoluteOrientation += 2 * M_PI;
    }
    X += deltaX;
    Y += deltaY;
    // x += globalDist * cos(globalAngle);
    // y += globalDist * sin(globalAngle);
    this_thread::sleep_for(5);
  }
  return 1;
}

// printstuff to controller
int ControllerPrint() {

  Brain.Timer.reset();
  // AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
  while (1) {
    Controller1.Screen.setCursor(1, 1);
    double speed = F1.velocity(pct);
    Controller1.Screen.print("Spd=%.2f tSpd=%.2f   ", speed, TargetSpeed);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("gAngle=%.2f ", GoalAngle);
    if (Brain.timer(sec) == 15) {
      Controller1.rumble(".");
    } // 2 minute mark
    if (Brain.timer(sec) == 45) {
      Controller1.rumble("..");
    } // 1:30 mark
    if (Brain.timer(sec) == 75) {
      Controller1.rumble("...");
    } // 1 minute mark
    if (Brain.timer(sec) == 105) {
      Controller1.rumble("....");
    } // 30 second mark
    this_thread::sleep_for(50);
  }
  }
    
/*
     Brain.Screen.printAt(1, 20, " prevHeading = %.2f ", prevHeading);
    Brain.Screen.printAt(1, 40, " deltaHeading = %.2f ", deltaHeading);
    Brain.Screen.printAt(1, 60, " absoluteOrientation = %.2f ", absoluteOrientation);
    Brain.Screen.printAt(1, 80, " localX = %.2f localY = %.2f ", localX, localY);
    Brain.Screen.printAt(1, 100, " lEncoder = %.2f bEncoder = %.2f", lEncoder, bEncoder);
    Brain.Screen.printAt(1, 120, " distL = %.2f distB = %.2f ", distL,distB);
    Brain.Screen.printAt(1, 140, " deltaX = %.2f deltaY = %.2f", deltaX, deltaY);
   Brain.Screen.printAt(1, 160, " averageHeading = %.2f ", averageHeading);
   this_thread::sleep_for(1000);
  }
  return (0);
} */

void controlFlywheelSpeed(double target) {
  double kI = .04;
  double speed = F1.velocity(pct);
  double error = target - speed;
  double fwDrive = FWDrive + kI * error;
  // :D
 // Brain.Screen.printAt(1, 40, " speed = %.2f ", speed);
  // Keep drive between 0 to 100%
  if (fwDrive > 100)
    fwDrive = 100;
  if (fwDrive < 0)
    fwDrive = 0;
  // Check for zero crossing
  if (error * OldError < 0) {
    fwDrive = 0.5 * (fwDrive + TBHval);
    TBHval = fwDrive;
  }

//  Brain.Screen.printAt(180, 40, "fwdrive %.1f  ", fwDrive);
  spinFlywheel(fwDrive);

  FWDrive = fwDrive;
  OldError = error;
}

void spinFlywheel(double speed) {
  speed = speed * 120; // speed is in pctage so convert to mV 100% = 12000
                       // mV
  F1.spin(forward, speed, voltageUnits::mV);
  F2.spin(forward, speed, voltageUnits::mV);
}

void flywheelMonitor() {
  double current1 = F1.current();
  double current2 = F2.current();
  double t1 = F1.temperature(fahrenheit);
  double t2 = F2.temperature(fahrenheit);
  double b = Brain.Battery.capacity();

  //Brain.Screen.printAt(1, 60, "F1 current = %.1f   Temp = %.1f   ", current1, t1);
  //Brain.Screen.printAt(1, 80, "F2 current = %.1f   Temp = %.1f   ", current2, t2);
 // Brain.Screen.printAt(1, 100, "Battery Capacity  = %.1f      ", b);
} //

// turret Pid spin to with gyro angle
int turretSpinTo(double targetAngle, bool global) {
  double kp = 1;
  double ki = 0;

  double kd = .5;
  double sum = 0;
  double prevError = 0;
  double error = targetAngle - turretG.orientation(yaw, degrees);
  double accuracy = 1;
  // while(true){
  double speed;
  while (fabs(error) > accuracy) {
    double turretEncoderAngle =
          (TurretE.angle() > 180 ? TurretE.angle() - 360 : TurretE.angle());
    if (global) {
      error = -(targetAngle - turretG.orientation(yaw, degrees));
    } else {
      error = targetAngle - turretEncoderAngle;
    }

    speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
   /* if ((speed > 0 && turretEncoderAngle < -90))
      speed = 0;
    if ((speed < 0 && turretEncoderAngle > 90))
      speed = 0;*/
    turret.spin(fwd, speed, pct);

    /*
    if (speed > 0) {
      turret.spin(fwd, -5, rpm);
    }
    if (speed < 0) {
      turret.spin(fwd, -5, pct);
    }*/

    wait(10, msec);
    prevError = error;
    sum += error;
  }
  if (fabs(error) < accuracy) {
    turret.stop();
  }
  //  }

  return 0;
}
bool loading = true;
void toggleTurret() {
  Controller1.rumble("."); 
  loading = !loading; 
  // wait(10, msec);
  // turretSpinTo(TargetAngle);
  // Controller1.rumble(".");
}

int turretStable() {
loading=true;
 // while (true) {
  double kp = 1.2;
  double ki = 0;

  double kd = .5;
  double sum = 0;
  double prevError = 0;
  
  double error = -GoalAngle - turretG.orientation(yaw, degrees);
  //double accuracy = 1;
  // while(true){
  double speed;
  while (true) {
    double turretEncoderAngle =
          (TurretE.angle() > 180 ? TurretE.angle() - 360 : TurretE.angle());
    if (!loading) {
      error = -GoalAngle - turretG.orientation(yaw, degrees);
    } else {
      error = turretEncoderAngle;
    }

    speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
    if ((speed > 0 && turretEncoderAngle < -90))
      speed = 0;
    if ((speed < 0 && turretEncoderAngle > 90))
      speed = 0;
    turret.spin(fwd, speed, pct);

    /*
    if (speed > 0) {
      turret.spin(fwd, -5, rpm);
    }
    if (speed < 0) {
      turret.spin(fwd, -5, pct);
    }*/

    wait(10, msec);
    prevError = error;
    sum += error;
  

    this_thread::sleep_for(5);  }
  return 0;
}
void pistonToggle() {

  Injector.set(true);
  wait(100, msec);
  Injector.set(false);
}
void pistonToggleReady() {
  Brain.Screen.drawRectangle(120, 190, 60, 60, orange);
  waitUntil(F1.velocity(pct) < TargetSpeed + .25 &&
            F1.velocity(pct) > TargetSpeed - .25);
  wait(10, msec);
  waitUntil(F1.velocity(pct) < TargetSpeed + .25 &&
            F1.velocity(pct) > TargetSpeed - .25);
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);

  Injector.set(true);
  wait(100, msec);
  Injector.set(false);
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
}
bool intakeOn = false;
void toggleIntake() { intakeOn = !intakeOn; }
bool driveDir=0;
void driveSwitch(){driveDir=!driveDir;}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  //sylib::initialize();
  if(!(RB.installed()&&LB.installed()&&RF.installed()&&LF.installed()&& //drive motors
  F1.installed()&&F2.installed() //flywheel
  &&Intake1.installed()&&turret.installed() //turret and intake
  &&gyro1.installed()&&RotationL.installed()&&RotationB.installed() //odom stuff
  &&turretG.installed()&&TurretE.installed()&&Vision16.installed()&&turretOptical.installed()&& //turret sensors
  Color.installed())) //roler sensor
    Controller1.rumble("--------------------------------------------------------------------------------------");

  vexcodeInit();
  // Initializing Robot Configuration. DO NOT REMOVE!
  thread odometeryTracking = thread(odometery);
  gyro1.calibrate();
  turretG.calibrate();
  waitUntil(!gyro1.isCalibrating() && !turretG.isCalibrating());
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here .   */
/*---------------------------------------------------------------------------*/

void autonomous(void) { spinFlywheel(100);
 wait(3, sec);
 Injector.set(!Injector.value());
 wait(300, msec);
 Injector.set(!Injector.value());

}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  X=0;
  Y=0;
  thread ControllerPrinting = thread(ControllerPrint);
  thread turretStablization = thread(turretStable);
  bool alg = true;
  gyro1.calibrate();
  turretG.calibrate();
  waitUntil(!gyro1.isCalibrating() && !turretG.isCalibrating());
  turretG.setHeading(180, degrees);
  while (true) {
    //target goal
    GoalAngle=atan2(X,122-Y)*(180/M_PI);
    /*if(!loading)TargetAngle=5
    ;
    else {
    TargetAngle=gyro1.orientation(yaw, degrees);
    }*/
    // if(Color.isNearObject()){
    //    turretSpinTo(0);

    //   }

  
    /*
        if (Controller1.ButtonL2.pressing() && !BumperL.pressing()) {
          // offset++;
          turret.spin(reverse, 30, pct);
          wait(10, msec);
        }
        if (Controller1.ButtonR2.pressing() && !BumperR.pressing()) {
          // offset--;
          turret.spin(forward, 30, pct);
          wait(10, msec);
        }*/

    // if (!Controller1.ButtonR2.pressing() && !Controller1.ButtonL2.pressing())
    // {
    //      turret.stop(brake);
    //   }
 

    /*if (Controller1.ButtonL2.pressing()) {
      GoalAngle -= 0.5;
      wait(10, msec);
    }
    if (Controller1.ButtonR2.pressing()) {
      GoalAngle += 0.5;
      wait(10, msec);
    }*/
    if (Controller1.ButtonL1.pressing()) {
      TargetSpeed -= 0.5;
      wait(10, msec);
    }
    if (Controller1.ButtonR1.pressing()) {
      TargetSpeed += 0.5;
      wait(10, msec);
    }
     //button controls
 //available buttons: X Y A
    if (Controller1.ButtonUp.pressing()&&TargetSpeed<100){
      TargetSpeed += 5;
      wait(10, msec);
    }
    if (Controller1.ButtonDown.pressing()&&TargetSpeed>0){
      TargetSpeed -= 5;
      wait(10, msec);
    }  
    
    //Brain.Screen.printAt(1, 20, "target speed = %.2f ", TargetSpeed);

    if (alg) {
      controlFlywheelSpeed(TargetSpeed);
     // Brain.Screen.printAt(1, 120, "controlled speed    ");
    } else {
      controlFlywheel1(TargetSpeed);
     // Brain.Screen.printAt(1, 120, "not controlled     ");
    }

    if (intakeOn) {
      Intake1.spin(forward, 150, rpm);
    }
    if (!intakeOn) {
      if (Color.color() == blue && Color.isNearObject()) {
        Intake1.spin(forward, 200, rpm);
      } else {

        Intake1.stop();
      }
    }
    Color.setLightPower(100);
    if (Color.isNearObject()) {
      Color.setLight(ledState::on);
    }
    if (!Color.isNearObject()) {
      Color.setLight(ledState::off);
    }
    if (F1.velocity(pct) < TargetSpeed + 1 &&
        F1.velocity(pct) > TargetSpeed - 1) {
      Brain.Screen.drawRectangle(60, 190, 60, 60, green);

    } else {
      Brain.Screen.drawRectangle(60, 190, 60, 60, red);
    }
    flywheelMonitor();
    // tank drive code
    if(driveDir){
    
    LF.spin(forward, Controller1.Axis3.position()*120, voltageUnits::mV);
    RF.spin(forward, Controller1.Axis2.position()*120, voltageUnits::mV);
    LB.spin(forward, Controller1.Axis3.position()*120, voltageUnits::mV);
    RB.spin(forward, Controller1.Axis2.position()*120, voltageUnits::mV);
    }
    else if (!driveDir) {
    LF.spin(reverse, Controller1.Axis2.position()*120, voltageUnits::mV);
    RF.spin(reverse, Controller1.Axis3.position()*120, voltageUnits::mV);
    LB.spin(reverse, Controller1.Axis2.position()*120, voltageUnits::mV);
    RB.spin(reverse, Controller1.Axis3.position()*120, voltageUnits::mV);
    }
    if (Controller1.Axis2.position() == 0 &&
        Controller1.Axis3.position() == 0) {
      LF.stop(coast);
      RF.stop(coast);
      LB.stop(coast);
      RB.stop(coast);
    }
    /*
    if (turretToggle == false) {

      if (gyro1.heading() - turretG.heading() > 90 &&
          gyro1.heading() - turretG.heading() < 270) {
        turretSpinTo(offset);
      } else if (gyro1.heading() - turretG.heading() > 90) {
        turretSpinTo(gyro1.heading() + 90);
      } else if (gyro1.heading() - turretG.heading() < 270) {
        turretSpinTo(gyro1.heading() - 90);
      }
    } else {

      if (gyro1.heading() < turretG.heading() + 1 &&
          gyro1.heading() > turretG.heading() + 1) {
        turret.stop(hold);
      } else {
        turretSpinTo(gyro1.heading());
      }
    }*/
    /*if (targetAngle != turretAngle) {
      if (targetAngle > 95) {
        targetAngle = 95;
      }
      if (targetAngle < -75) {
        targetAngle = 75;
      }
      turretSpinTo(targetAngle);
    }
    turretG.orientation(yaw, degrees);*/

    wait(10, msec);
  }
}

// Main will set up the competition functions and callbacks.
//
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Controller1.ButtonDown.pressed(turretStable);

  Controller1.ButtonB.pressed(toggleIntake);
  Controller1.ButtonLeft.pressed(pistonToggle);
  Controller1.ButtonRight.pressed(driveSwitch);
  Controller1.ButtonX.pressed(toggleTurret);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
