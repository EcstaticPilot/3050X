

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
