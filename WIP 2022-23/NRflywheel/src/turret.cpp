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
int mode = 0;
extern bool isRed;
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

  double error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
  // double accuracy = 1;
  // while(true){
  double speed;
  while (true) {
    GoalAngle = atan2(X - 115, 115 - Y) * (180 / M_PI);

    if (!loading) {

      if (fabs(GoalAngle + offset - turretG.orientation(yaw, degrees)) < 300) {
        if (isRed)
          Vision16.takeSnapshot(RGOAL);
        else
          Vision16.takeSnapshot(BGOAL);
        // if goal is within limits
        // use vision sensor
        // find closest
        int object0 = abs(140 - Vision16.objects[0].centerX);
        int object1 = abs(140 - Vision16.objects[1].centerX);
        int object2 = abs(140 - Vision16.objects[2].centerX);
        int object3 = abs(140 - Vision16.objects[3].centerX);
        int object4 = abs(140 - Vision16.objects[4].centerX);
        int object5 = abs(140 - Vision16.objects[5].centerX);
        int object6 = abs(140 - Vision16.objects[6].centerX);
        int object7 = abs(140 - Vision16.objects[7].centerX);
        int visionObjects[8] = {object0, object1, object2, object3,
                                object4, object5, object6, object7};
        //try this? https://www.sanfoundry.com/cpp-program-minimum-element-array-using-linear-search-2/
        std::min_element(visionObjects[0], visionObjects[7]);
        int result = std::min_element(visionObjects[0], visionObjects[7]);
        int i = 0;
        while (!(visionObjects[i] == result)) {
          i++;
        }
        error = Vision16.objects[i].centerX - 140;

        Brain.Screen.printAt(1, 100, "error  = %.1f      ", error);
        Brain.Screen.printAt(1, 160, "vision  = %.1f      ",
                             Vision16.largestObject.centerX);
        mode = 1;
        kp = 0.5;
        kd = .3;
        if (fabs(error) < 20)
          VisionReady = true;
        else
          VisionReady = false;
      } else {
        VisionReady = false;
        error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
        kp = 1;
        kd = 0.3;
        mode = 2;
      }
      //    VisionReady=false;//vision is not ready
    } else { // if not loading go to zero

      error = TurretE.position(turns) * 360;
      std::cout << error << std::endl;
      kp = 0.5;
      kd = 0.3;
      mode = 4;
      VisionReady = false; // vision is not ready
    }

    speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
    if ((speed > 0 && TurretE.position(turns) * 360 < -120))
      speed = 0;
    if ((speed < 0 && TurretE.position(turns) * 360 > 180))
      speed = 0;
    turret.spin(fwd, speed, pct);

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
