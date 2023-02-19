#include "vex.h"
#include "vision.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

bool loading=false;
double GoalAngle;
extern double X, Y;
float offset = 0;
bool VisionReady;
int mode = 0;
extern bool isRed;

int findClosetObject(int a, int b){
  if(abs(140-Vision16.objects[a].centerX)
  <abs(140-Vision16.objects[b].centerX))
  {
    return a;
  }
  else {
    return b;
  }
}

int turretStable() {
  Controller1.rumble("..");
  loading = true;
  TurretE.setPosition(0, turns);

  // safe working values double kp = 1; double ki = 0;double kd = 0.3;
  double kp = 3;
  double ki = 0;
  double kd = 0.9;
  double sum = 0;
  double prevError = 0;
  double error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
  // double accuracy = 1;
  double speed;

  while (true) {
    GoalAngle = atan2(X - 100, 120 - Y) * (180 / M_PI);

    if (!loading) {
      std::cout<<GoalAngle- turretG.orientation(yaw, degrees)<<std::endl;
      if (fabs(GoalAngle- turretG.orientation(yaw, degrees)) < 50) {
        if(isRed) { 
          Vision16.takeSnapshot(RGOAL);
        }
        else {
          Vision16.takeSnapshot(BGOAL);
        }
        if (Vision16.largestObject.exists) {
          error = Vision16.largestObject.centerX - 140;

          Brain.Screen.printAt(1, 100, "error  = %.1f      ", error);
          Brain.Screen.printAt(1, 160, "vision  = %.1f      ", Vision16.largestObject.centerX);

          mode = 1;
          kp = 0.2;

          if(fabs(error)<20) VisionReady=true;
          else VisionReady=false;
        } 
        else {
          VisionReady=false;
          error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
          kp = 1;
          kd = 0.3;
          mode=2;
          GoalAngle = atan2(X - 115, 115 - Y) * (180 / M_PI);

////////////////////////////////////////////////////////////////

    if (!loading) {

      if (fabs(GoalAngle + offset - turretG.orientation(yaw, degrees)) < 300) {
        if (isRed) {
          Vision16.takeSnapshot(RGOAL);
        }
        else {
          Vision16.takeSnapshot(BGOAL);
        }
        int closestObject=0;

        for(int i=0; i<5; ++i) {
          closestObject=findClosetObject(closestObject, i);
        }
        //try this? https://www.sanfoundry.com/cpp-program-minimum-element-array-using-linear-search-2/
     
        error = Vision16.objects[closestObject].centerX - 140;

        Brain.Screen.printAt(1, 100, "error  = %.1f      ", error);
        Brain.Screen.printAt(1, 160, "vision  = %.1f      ", Vision16.largestObject.centerX);

        mode = 1;
        kp = 0.5;
        kd = .3;

        if (fabs(error) < 20)
          VisionReady = true;
        else
          VisionReady = false;
      } 
      else {
        VisionReady = false;
        error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
        kp = 1;
        kd = 0.3;
        mode = 2;
      }
    } 
    else { // if not loading go to zero
      error = TurretE.position(turns)*360;
      kp = 0.5;
      kd = 0.3;
      mode=4;
      VisionReady = false; 
    }

      speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
      if ((speed > 0 && TurretE.position(turns)*360 < -120))
        speed = 0;
      if ((speed < 0 && TurretE.position(turns)*360 > 180))
        speed = 0;
      turret.spin(fwd, speed, pct);

      prevError = error;
      sum += error;
      this_thread::sleep_for(10);

      error = TurretE.position(turns) * 360;
      std::cout << error << std::endl;
      kp = 0.5;
      kd = 0.3;
      mode = 4;
      VisionReady = false; // vision is not ready
    }

    // speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
    // if ((speed > 0 && TurretE.position(turns) * 360 < -120))
    //   speed = 0;
    // if ((speed < 0 && TurretE.position(turns) * 360 > 180))
    //   speed = 0;
    // turret.spin(fwd, speed, pct);

    // prevError = error;
    // sum += error;
    // this_thread::sleep_for(10);
  }
  return 1;
}
}
}

void turretSpinTo(double targetAngle, bool global) {
  double kp = 1;
  double ki = 0;
  double kd = .5;
  double sum = 0;
  double prevError = 0;
  double error = targetAngle - turretG.orientation(yaw, degrees);
  double accuracy = 1;
  double speed;

  while (fabs(error) > accuracy) {
    double turretEncoderAngle = 
    (TurretE.angle() > 180 ? TurretE.angle() - 360 : TurretE.angle());
    if (global) {
      error = -targetAngle - turretG.orientation(yaw, degrees);
    } 
    else {
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
}

void toggleTurret() {
  Controller1.rumble(".");
  loading = !loading;
}
