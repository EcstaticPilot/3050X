
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       NR 7700P                                                  */
/*    Created:      May 13, 2022                                              */
/*    Description:  code of Nikhil Ramanuja 7700P turret bot vex spin up      */
/*----------------------------------------------------------------------------*/
// vex::vision::signature BGOAL = vex::vision::signature (1, -1855, 2553, 349,
// 2243, 10039, 6141, 1, 0);
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// F1                   motor         2
// F2                   motor         15
// Injector             digital_out   A
// LF                   motor         18
// LB                   motor         12
// RF                   motor         20
// RB                   motor         4
// Intake1              motor         1
// turret               motor         21
// gyro1                inertial      11
// RotationL            rotation      5
// RotationB            rotation      3
// turretG              inertial      14
// Color                optical       7
// TurretE              rotation      17
// turretOptical        optical       9
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "stdio.h"
#include "vex.h"
#include "vision.h"
#include <math.h>

//#include "sylib.hpp"

using namespace vex;
// 100 digits of pi because I like Pi𝝿

// std::string team = "blue";
// A global instance of competition

competition Competition;

void flywheelMonitor();
void spinFlywheel(double);


double OldError = 0.0;
double TBHval = 0.0;

double FWDrive = 0.0;
double GoalAngle;
float offset = 0;
bool loading = true;
double TargetSpeed = 0.0;
long double pi = 3.14159265358979323;
float C = pi * 3.25;
bool TurretToggle = false;

void drive(int lSpeed, int rSpeed, double wt) {
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

void drive_brake() {

  LF.stop(brake);
  RF.stop(brake);
  LB.stop(brake);
  RB.stop(brake);
}
/*

ODOMETRY

*/

double X = 75;double Y = 0; // declare global x and y

int odometery() {
double prevHeading = gyro1.heading();
double deltaHeading = 0; // change in heading
double absoluteOrientation = M_PI;
double lRad = 1.375;      // radius of tracking wheel
double bRad = 1.375;      // radius of tracking wheel
double lEncoder = 0;      // declaring encoder variable left
double bEncoder = 0;      // declaring encoder variable back
double distL = 0;         // distance left encoder has traveled
double distB = 0;         // distance back encoder has traveled
double prevLE = lEncoder; // create previous encoder value left
double prevBE = bEncoder; // create previous encoder value back
double averageHeading;    //
double deltaX;
double deltaY;
  Controller1.rumble(".");
  RotationL.resetPosition();
  RotationB.resetPosition();
  while (1) {
    lEncoder = RotationL.position(degrees);
    bEncoder = RotationB.position(degrees);

    distL = ((lEncoder - prevLE) * M_PI / 180) * lRad;
    // convert encoder distance into distance traveled
    distB = ((bEncoder - prevBE) * M_PI / 180) * bRad;
    // convert encoder distance into disntance traveled

    prevLE = lEncoder; // create previous encoder value left
    prevBE = bEncoder; // create previous encoder value back

    absoluteOrientation = gyro1.heading(degrees) * M_PI / 180.0;
    deltaHeading =
        absoluteOrientation - prevHeading; // calculate change in heading
    averageHeading = prevHeading + (deltaHeading) / 2;
    prevHeading = absoluteOrientation;

    deltaX = (distL * sin(averageHeading)) + (distB * cos(averageHeading));
    deltaY = (distL * cos(averageHeading)) - (distB * sin(averageHeading));
    while (absoluteOrientation >= 2 * M_PI) {
      absoluteOrientation -= 2 * M_PI;
    }

    while (absoluteOrientation < 0) {
      absoluteOrientation += 2 * M_PI;
    }
    X += deltaX;
    Y += deltaY;

    this_thread::sleep_for(5);
  }
  return 1;
}

/*

CONTROLLER PRINTING

*/
int ControllerPrint() {

  Brain.Timer.reset();
  
  while (1) {
    Controller2.Screen.setCursor(1, 1);
    double speed = F1.velocity(pct);
    Controller2.Screen.print("Spd=%.2f tSpd=%.2f   ", speed, TargetSpeed);
    Controller2.Screen.setCursor(2, 1);
    Controller2.Screen.print("pos= (%.1f,%.1f)", X, Y);
    Controller2.Screen.setCursor(3, 1);
    Controller2.Screen.print("time=%.2f ", Brain.timer(sec));

    if (Brain.timer(sec) == 15) {
      Controller1.rumble(".");
    } // 2 minute mark
    if (Brain.timer(sec) == 45) {
      Controller1.rumble("..");
    } // 1:30 mark
    if (Brain.timer(sec) == 75)
      Controller1.rumble("...");
    // 1 minute mark
    if (Brain.timer(sec) == 105) {
      Controller1.rumble("....");
    } // 30 second mark

    this_thread::sleep_for(50);
  }
}

/*

FLYWHEEL CONTROL

*/

void spinFlywheel(double speed) {
  speed = speed * 120; // speed is in pctage so convert to mV 100% = 12000
                       // mV
  F1.spin(forward, speed, voltageUnits::mV);
  F2.spin(forward, speed, voltageUnits::mV);
}

int controlFlywheelSpeed() {
  double kp = .04;
  while (true) {
    
        double speed = F1.velocity(pct);
        double error = TargetSpeed - speed;
     //   double fwDrive = FWDrive + kI * error;
        // :D
        // Brain.Screen.printAt(1, 40, " speed = %.2f ", speed);
        // Keep drive between 0 to 100%
    /*
        if (error > 20) {
          fwDrive = 100;
        }

        else {
          if (fwDrive > 100)
            fwDrive = 100;
          if (fwDrive <= 0)
            fwDrive = 0;
          // Check for zero crossing
          if (error * OldError < 0) {
            fwDrive = 0.5 * (fwDrive + TBHval);
            TBHval = fwDrive;
          }
      //  }
              */
    //  Brain.Screen.printAt(180, 40, "fwdrive %.1f  ", fwDrive);
    spinFlywheel(TargetSpeed+kp*error);

    // FWDrive = fwDrive;
    // OldError = error;
  }
  return 1;
}

/*

TURRET CONTROL

*/

void turretSpinTo(double targetAngle, bool global) {
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
      error = -targetAngle - turretG.orientation(yaw, degrees);
    } else {
      error = targetAngle - turretEncoderAngle;
    }

    speed = (error * kp) + (ki * sum) + (kd * (error - prevError));

    turret.spin(fwd, speed, pct);

    wait(10, msec);
    prevError = error;
    sum += error;
  }
  if (fabs(error) < accuracy) {
    turret.stop();
  }
  //  }
}

void toggleTurret() {
  Controller1.rumble(".");
  loading = !loading;
}

bool VisionReady = false;
int turretStable() {
  Controller1.rumble("..");
  loading = true;
  // while (true) {
  // safe working valuse double kp = 1; double ki = 0;double kd = 0.3;
  double kp = 1;
  double ki = 0;
  double kd = 0.4;
  double sum = 0;
  double prevError = 0;

  double error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
  // double accuracy = 1;
  // while(true){
  double speed;
  while (true) {
    GoalAngle = atan2(X - 115, 115 - Y) * (180 / M_PI);
    double turretEncoderAngle =
        (TurretE.angle() > 180 ? TurretE.angle() - 360 : TurretE.angle());
    if (!loading) {

      if (fabs(GoalAngle + offset - turretG.orientation(yaw, degrees)) <
          3) {                        // if goal is within limits
        Vision16.takeSnapshot(RGOAL); // use vision sensor
        if (Vision16.largestObject.exists) {

          error = Vision16.largestObject.centerX - 158;

          Brain.Screen.printAt(1, 100, "error  = %.1f      ", error);
          Brain.Screen.printAt(1, 160, "vision  = %.1f      ",
                               Vision16.largestObject.centerX);
          kp = 0.2;
          kd = 0;
          if(fabs(error)<10)VisionReady=true;
          else VisionReady=false;
        } else {
          VisionReady=false;
          error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
          kp = 0.8;
          kd = 0.2;
        }
      } else {
        VisionReady=false;
        error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
        kp = 0.8;
        kd = 0.2;
      }
      //    VisionReady=false;//vision is not ready
          }
      else { // if not loading go to zero

        error = turretEncoderAngle;
        kp = 1;
        kd = 0.4;
        VisionReady = false; // vision is not ready
      }

      speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
      if ((speed > 0 && turretEncoderAngle < -150))
        speed = 0;
      if ((speed < 0 && turretEncoderAngle > 85))
        speed = 0;
      turret.spin(fwd, speed*12,volt);

      /*
      if (speed > 0) {
        turret.spin(fwd, -5, rpm);
      }
      if (speed < 0) {
        turret.spin(fwd, -5, pct);
      }*/

      prevError = error;
      sum += error;
      this_thread::sleep_for(10);
    }
     return 1;
  }
 
/*

PISTON CONTROL

*/

void pistonToggle() {
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
}

void fireDisc() {
  loading = false;
  TargetSpeed = 0; // need to create formula
  waitUntil((fabs(GoalAngle - turretG.orientation(yaw, degrees)) < 2) &&
            (fabs(F1.velocity(pct) - TargetSpeed) < .25));
  pistonToggle();
  if (turretOptical.isNearObject())
    fireDisc();
  else {

    loading = true;
    TargetSpeed = 0;
  }
}

void pistonToggleReady() {
  Brain.Screen.drawRectangle(120, 190, 60, 60, orange);
  waitUntil(F1.velocity(pct) < TargetSpeed + .25 &&
            F1.velocity(pct) > TargetSpeed - .25);
  wait(10, msec);
  waitUntil(F1.velocity(pct) < TargetSpeed + .25 &&
            F1.velocity(pct) > TargetSpeed - .25);
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
  if (turretOptical.isNearObject()) {
    Injector.set(true);
    wait(100, msec);
    Injector.set(false);
  }
  Brain.Screen.drawRectangle(120, 190, 60, 60, black);
}
bool intakeOn = false;
void toggleIntake() { intakeOn = !intakeOn; }
bool driveDir = 0;
void driveSwitch() { driveDir = !driveDir; }

/*---------------------------------------------------------------------------*/
/*                          Pre-omousomous Functions */
/*                                                                           */
/*  You may want to perform some actions before the competition starts. */
/*  Do them in the following function.  You must return from this function */
/*  or the autonomous and usercontrol tasks will not be started.  This */
/*  function is only called once after the V5 has been powered on and */
/*  not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // sylib::initialize();
  vexcodeInit();
  if (!(RB.installed() && LB.installed() && RF.installed() &&
        LF.installed() &&                            // drive motors
        F1.installed() && F2.installed()             // flywheel
        && Intake1.installed() && turret.installed() // turret and intake
        && gyro1.installed() && RotationL.installed() &&
        RotationB.installed() // odom stuff
        && turretG.installed() && TurretE.installed() && Vision16.installed() &&
        turretOptical.installed() && // turret sensors
        Color.installed()))          // roler sensor
    Controller1.rumble("-------------------------------------------------------"
                       "-------------------------------");

  // Initializing Robot Configuration. DO NOT REMOVE!
  thread odometeryTracking = thread(odometery);
  thread flywheelgo = thread(controlFlywheelSpeed);
  gyro1.calibrate();
  turretG.calibrate();
  waitUntil(!gyro1.isCalibrating() && !turretG.isCalibrating());
 // Brain.Screen.pressed(teamSwitch);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of */
/*  a VEX Competition. */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here . */
/*---------------------------------------------------------------------------*/

void rotate(double dir, double accuracy = 1) {
  // double currDir = gyro1.rotation(degrees);
  double speed = 100;
  double error = dir;
  double prevError = dir;
  double Kd = 1;
  double Ki = 0.2;
  double sum = 0;
  double Kp = .8;

  // change facing
  // dir = currDir - gyro1.rotation(degrees); // rotation offset

  // continues rotating until its within accuracy degrees from the target and
  // the speed of the motors isn't too fast
  while (fabs(error) > accuracy) {
    error = dir - gyro1.rotation(degrees);
    speed = (Kp * error) + (Ki * sum) + (Kd * (error - prevError));
    LB.spin(fwd, speed / 3, percent);
    RB.spin(reverse, speed / 3, percent);
    RF.spin(reverse, speed / 3, percent);
    LF.spin(fwd, speed / 3, percent);

    wait(10, msec);
    prevError = error;
    sum = sum * 0.5 + error;
  }

  LF.stop(brake);
  RF.stop(brake);
  RB.stop(brake);
  LB.stop(brake);
}

void inchDrive(float dist, float accuracy = 1) {

  float start_position = LF.position(rev);
  float curr_position = 0.0;
  float error = dist - curr_position;
  float old_error = error;
  float kp =
      10.0; // <-- how many - before the target you want to start slow down
  float ki = 2.0; // <-- why do i need this??
  float kd = 5.0;
  float speed = 100;
  float sum = 0.0;

  while (fabs(error) > accuracy) {

    drive(speed, speed, 10); // move at 100% for 10 millisecs
    curr_position = (LF.position(rev) - start_position) *
                    C; // total inches traveled since start position

    old_error = error;            // save error
    error = dist - curr_position; // get current error

    sum = sum * 0.7 + error; // <--
    speed = (error * kp) /*-*/ +
            (ki * sum) /*speeds u u if ur taking a long time*/ +
            (kd * (error - old_error)); /*slows u down
if you're going too fast
*/
  }
  drive_brake();
}



void autonomous(void) {
  // thread turretStablization = thread(turretStable);
/*

  inchDrive(0.3);
  rotate(90);
  inchDrive(24);
  rotate(180);
  inchDrive(0.1);
  Intake1.spin(forward, 100, pct);
  waitUntil(Color.color() == blue);
  Intake1.stop();&*/

  /*
   inchDrive(0.3);
   rotate(-90);
   inchDrive(24);
   rotate(-90);
   inchDrive(0.1);
   Intake1.spin(forward, 100, pct);
   waitUntil(Color.color() == red);
   Intake1.stop();
   */

  turretSpinTo(atan2(X - 110, 110 - Y) * (180 / M_PI), true);
  GoalAngle = atan2(X - 115, 115 - Y) * (180 / M_PI);
  loading = false;
  spinFlywheel(100);
  loading = false;
  waitUntil(F1.velocity(pct) > 99&&F1.velocity(pct)<101);
  pistonToggle();
  loading = false;
  waitUntil(F1.velocity(pct) > 99&&F1.velocity(pct)<101);
  pistonToggle();
  loading = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of
 */
/*  a VEX Competition. */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  X = 75;
  Y = 0;
  thread ControllerPrinting = thread(ControllerPrint);
  thread turretStablization = thread(turretStable);
  thread flywheelgo = thread(controlFlywheelSpeed);

  
  turretG.setHeading(180, degrees);
  while (true) {

    /*

    CONTROLLER 2 SPEED CONTROL

    */
    if (Controller2.ButtonL1.pressing()) {
      TargetSpeed -= 0.5;
      wait(10, msec);
    }
    if (Controller2.ButtonR1.pressing()) {
      TargetSpeed += 0.5;
      wait(10, msec);
    }
    if (Controller2.ButtonL2.pressing()) {
      offset += 0.5;
      wait(10, msec);
    }
    if (Controller2.ButtonR2.pressing()) {
      offset -= 0.5;
      wait(10, msec);
    }

    if (Controller2.ButtonX.pressing()) {
      TargetSpeed = 0;
    }
    if (Controller2.ButtonA.pressing()) {
      TargetSpeed = 75;
    }
    if (Controller2.ButtonB.pressing()) {
      TargetSpeed = 85;
    }
    if (Controller2.ButtonY.pressing()) {
      TargetSpeed = 100;
    }
    /*

    INTAKE

    */
    if (intakeOn) {
      Intake1.spin(forward, 150, rpm);
    } else {

      if (Color.color() == red  && Color.isNearObject())
        Intake1.spin(forward, 200, rpm);
      else
        Intake1.stop();
    }
    Color.setLightPower(100);
    if (Color.isNearObject())
      Color.setLight(ledState::on);
    else
      Color.setLight(ledState::off);
    /*

    TANK DRIVE CODE

    */
    if (driveDir) {

      LF.spin(forward, Controller1.Axis3.position() * 120, voltageUnits::mV);
      RF.spin(forward, Controller1.Axis2.position() * 120, voltageUnits::mV);
      LB.spin(forward, Controller1.Axis3.position() * 120, voltageUnits::mV);
      RB.spin(forward, Controller1.Axis2.position() * 120, voltageUnits::mV);
    } else if (!driveDir) {
      LF.spin(reverse, Controller1.Axis2.position() * 120, voltageUnits::mV);
      RF.spin(reverse, Controller1.Axis3.position() * 120, voltageUnits::mV);
      LB.spin(reverse, Controller1.Axis2.position() * 120, voltageUnits::mV);
      RB.spin(reverse, Controller1.Axis3.position() * 120, voltageUnits::mV);
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
  Controller1.ButtonLeft.pressed(pistonToggleReady);
  Controller1.ButtonRight.pressed(driveSwitch);
  Controller1.ButtonX.pressed(toggleTurret);
  Controller2.ButtonUp.pressed(pistonToggle);
 // Controller2.ButtonDown.pressed(down);
  Controller2.ButtonLeft.pressed(pistonToggle);
 // Controller2.ButtonRight.pressed(rightB);
  // Controller1.ButtonX.pressed(fireDisc);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
