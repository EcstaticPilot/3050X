#include "vex.h"
#include <math.h>
#include "vision.h"
bool loading=false;
double GoalAngle;
extern double X,Y;
float offset=0;
bool VisionReady;
int turretStable() {
  Controller1.rumble("..");
  loading = true;
  // while (true) {
  // safe working valuse double kp = 1; double ki = 0;double kd = 0.3;
  double kp = 0.6;
  double ki = 0;
  double kd = 0.2;
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
          kp = 0.6;
          kd = 0.2;
        }
      } else {
        VisionReady=false;
        error = -(GoalAngle + offset) - turretG.orientation(yaw, degrees);
        kp = 0.6;
        kd = 0.2;
      }
      //    VisionReady=false;//vision is not ready
          }
      else { // if not loading go to zero

        error = turretEncoderAngle;
        kp = .6;
        kd = 0.2;
        VisionReady = false; // vision is not ready
      }

      speed = (error * kp) + (ki * sum) + (kd * (error - prevError));
      if ((speed > 0 && turretEncoderAngle < -150))
        speed = 0;
      if ((speed < 0 && turretEncoderAngle > 85))
        speed = 0;
      turret.spin(fwd, speed*1.2 , volt);
      

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
