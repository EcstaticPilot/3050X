
#include "stdio.h"
#include "vex.h"

#include <math.h>

float C = M_PI * 3.25;

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

void rotate(double dir, double accuracy = 1) {
  // double currDir = gyro1.rotation(degrees);
  double speed = 100;
  double error = dir - gyro1.rotation(degrees);
  double prevError = dir;
  double Kd = 1;
  double Ki = 0.6;
  double sum = 0;
  double Kp = .8;
  Brain.Screen.clearScreen();
  // change facing
  // dir = currDir - gyro1.rotation(degrees); // rotation offset

  // continues rotating until its within accuracy degrees from the target and
  // the speed of the motors isn't too fast
  while (fabs(error) > accuracy) {
    speed = (Kp * error) + (Ki * sum) + (Kd * (error - prevError));
    LB.spin(fwd, speed / 3, percent);
    RB.spin(reverse, speed / 3, percent);
    RF.spin(reverse, speed / 3, percent);
    LF.spin(fwd, speed / 3, percent);

    wait(10, msec);
    prevError = error;
    sum = sum * 0.5 + error;
    error = dir - gyro1.rotation(degrees);
  }
  drive_brake();
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
            (ki * sum) /*speeds u up if ur taking a long time*/ +
            (kd * (error - old_error)); /*slows u down
if you're going too fast
*/
  }
  drive_brake();
}
extern double X, Y;
void DriveToPoint(double targetX, double targetY, float speedMult = 1) {
  // T=turn
  // D=distance
  float tError = atan2(X - targetX, targetY - Y);
  float Tkp = .4, Tki = .2, Tkd = .33;
  float tErrorOld=tError;
  float tSum=0;

  float dError =
      sqrt((targetX - X) * (targetX - X) + (targetY - Y) * (targetY - Y));
  float Dkp = 10, Dki = 2, Dkd = 5;
  float dErrorOld=dError;
  float dSum=0;

  float accuracy = 3;

  while (!((fabs(targetX - X) < accuracy) && (fabs(targetY - Y) < accuracy))) {
    tError = atan2(X - targetX, targetY - Y);
    dError =
        sqrt((targetX - X) * (targetX - X) + (targetY - Y) * (targetY - Y));
    float lSpeed = (Dkp * dError) + (Dki * dSum) + (Dkd * (dError - tErrorOld)) + 
    (Tkp * tError) + (Tki * tSum) + (Tkd * (tError - tErrorOld));

    float rSpeed=(Dkp * dError) + (Dki * dSum) + (Dkd * (dError - dErrorOld)) -( 
    (Tkp * tError) + (Tki * tSum) + (Tkd * (tError - tErrorOld)));
    drive((lSpeed) * speedMult, (rSpeed) * speedMult, 10);
    dErrorOld=dError;
    dSum+=dError;

    tErrorOld=tError;
    tSum+=tError;
  }
  drive_brake();
}