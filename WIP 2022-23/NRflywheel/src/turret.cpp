
#include "vex.h"
#include "vision.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>
bool loading = false;
double GoalAngle;
extern double X, Y;
float offset = 0;
bool VisionReady;
extern bool Far_Side;
extern competition Competition;
int mode = 0;
extern bool isRed;
int findClosetObject(int a, int b) {
  if (!Vision16.objects[b].exists) {
    return a;
  }
  if(Vision16.objects[b].centerY<50){
    return a;
  }
  if (abs(150 - Vision16.objects[a].centerX) <
      abs(150 - Vision16.objects[b].centerX)) {
    return a;
  } else {
    return b;
  }
}
double joyAngle;
int turretStable() {
  Controller1.rumble("..");
  loading = true;
  TurretE.setPosition(0, turns);
  // while (true) {
  // safe working valuse double kp = 1; double ki = 0;double kd = 0.3;
  double kp = 3;
  double ki = 0;
  double kd = 0.9;
  double sum = 0;
  double prevError = 0;

  double error = -(GoalAngle)-turretG.orientation(yaw, degrees);
  // double accuracy = 1;
  // while(true){
  double speed;
  if(Far_Side){
  turretG.setHeading(90, degrees);
  }
  while (true) {
    // GoalAngle = atan2(X - 115, 115 - Y) * (180 / M_PI);
    if (Competition.isDriverControl()) {
      joyAngle =
          atan2(Controller2.Axis1.position(), Controller2.Axis2.position()) *
          180 / M_PI;

      if ((abs(Controller2.Axis1.position()) < 5) &&
          (abs(Controller2.Axis2.position()) < 5))
        loading = true;
      else
        loading = false;
    }
    
    if (!loading) {

      if (fabs(joyAngle - (turretG.heading() > 180 ? turretG.heading() - 360 : turretG.heading())
                                    )<40) {
        if (isRed)
          Vision16.takeSnapshot(RGOAL);
        else
          Vision16.takeSnapshot(BGOAL);
        // if goal is within limits
        // use vision sensor
        // find closest

        int closestObject = 0;
        // int i=0;
        for (int i = 0; i <= Vision16.objectCount; ++i) {
          // while(i<7){
          closestObject = findClosetObject(closestObject, i);
          // i++;
        }
        // try this?
        // https://www.sanfoundry.com/cpp-program-minimum-element-array-using-linear-search-2/

        error = Vision16.largestObject.centerX - (155 + offset);
        if (fabs(error) > 50) {

          error = joyAngle - (turretG.heading() > 180 ? turretG.heading() - 360 : turretG.heading());

          kp = 1;
          kd = 0.3;
          mode = 2;
        } else {

          Brain.Screen.printAt(1, 100, "error  = %.1f      ", error);
          Brain.Screen.printAt(1, 160, "vision  = %.1f      ",
                               Vision16.largestObject.centerX);
          mode = 1;
          kp = 0.15;
          kd = .0;
          if (fabs(error) < 15)
            Controller2.rumble(".");
          else
            VisionReady = false;
        }

        //    if (Competition.isAutonomous()) error = -GoalAngle -
        //    turretG.orientation(yaw, degrees);
      } else {
        // if(competition.){statements
        // }

        error = joyAngle - (turretG.heading() > 180 ? turretG.heading() - 360 : turretG.heading());

        kp = 1;
        kd = 0.3;
        mode = 2;
      }
      //    VisionReady=false;//vision is not ready
    } else { // if not loading go to zero

      error = TurretE.position(turns) * 360*3/7;//gear ratio
      std::cout << error << std::endl;
      kp = 1;
      kd = 0.3;
      mode = 4;
      VisionReady = false; // vision is not ready
    }
    
    speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
    if ((speed > 0 && TurretE.position(turns) * 360 < -240))
      speed = 0;
    if ((speed < 0 && TurretE.position(turns) * 360 > 140)){
      speed = 0;
      turret.stop(brake);
    }
      
    // if(fabs(speed)<1)
    // turret.stop(hold);
    //  else
    turret.spin(fwd, speed + 0.5*(speed - turret.velocity(pct)), pct);

    prevError = error;
    sum += error;
    this_thread::sleep_for(10);
  }
  return 1;
}

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
