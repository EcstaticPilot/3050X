
#include "vex.h"
#include <iostream>
#include <stdio.h>

#include <math.h>

float C = M_PI * 3.25;

void drive(int lSpeed, int rSpeed, double wt) {
  LF.spin(forward, lSpeed, pct);
  RF.spin(forward, rSpeed, pct);
  LB.spin(forward, lSpeed, pct);
  RB.spin(forward, rSpeed, pct);
  wait(wt, msec);
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
  float tError = gyro1.rotation(degrees) - atan2(targetY - Y, X - targetX);
  float Tkp = 1, Tki = 0, Tkd = 1;
  float tErrorOld = tError;
  float tSum = 0;

  float dError =
      sqrt((targetX - X) * (targetX - X) + (targetY - Y) * (targetY - Y));
  float Dkp = 0, Dki = 0, Dkd = 0;
  float dErrorOld = dError;
  float dSum = 0;

  float accuracy = 1;

  while ((fabs(targetX - X) > accuracy) || (fabs(targetY - Y) > accuracy)) {

    tError = -gyro1.orientation(yaw, degrees) -
             (atan2(targetY - Y, X - targetX) * 180 / M_PI);
    dError =
        sqrt((targetX - X) * (targetX - X) + (targetY - Y) * (targetY - Y));
    float rSpeed = (Dkp * dError) + (Dki * dSum) +
                   (Dkd * (dError - tErrorOld)) + (Tkp * tError) +
                   (Tki * tSum) + (Tkd * (tError - tErrorOld));

    float lSpeed =
        (Dkp * dError) + (Dki * dSum) + (Dkd * (dError - dErrorOld)) -
        ((Tkp * tError) + (Tki * tSum) + (Tkd * (tError - tErrorOld)));
    if (speedMult < 0) {
      double idk;
      idk = rSpeed;
      rSpeed = lSpeed;
      lSpeed = idk;
    }
    std::cout << lSpeed << "," << rSpeed << "," << dError <<","<<tError<< std::endl;
    drive((lSpeed)*speedMult, (rSpeed)*speedMult, 12);
    dErrorOld = dError;
    dSum += dError;

    tErrorOld = tError;
    tSum += tError;
  }
  drive_brake();
}
void DriveToPoint2(float targetX,float targetY){

  rotate((atan2(targetY - Y, X - targetX) * 180 / M_PI));
  std::cout << gyro1.rotation()<< std::endl;
  wait(1, sec);
  inchDrive(-(sqrt((targetX - X) * (targetX -  X) + (targetY - Y) * (targetY - Y))));
  std::cout << X << "," << Y << std::endl;
}
void RAMSETE(float targetX, float targetY, float targetAngle,
             float accuracy = 1) {
  // https://wiki.purduesigbots.com/software/control-algorithms/ramsete
  //θ - copy and paste theta
  float errorX = targetX - X;
  float errorY = targetY - Y;
  float smallScalar = 0.01;
  float beta = 0.3;
  float zeta = .2;
  while (!((fabs(errorX) < accuracy) && (fabs(errorY)) < accuracy)) {
    errorX = targetX - X;
    errorY = targetY - Y;
    float errorθ = - gyro1.rotation()-(atan2(targetY - Y, X - targetX) * 180 / M_PI);
    float ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
    float ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
    float eθ = errorθ;

    float vd = errorY*smallScalar;
    float wd = eθ*smallScalar;
    float k = 2 * zeta * sqrt(pow(wd, 2) + beta * pow(vd, 2));
    float v = (vd * cos(eθ)) + (k * ey);
    float w = wd + k * eθ + (beta * vd * sin(eθ) * ex) / eθ;
    
    float linearMotorVelocity = v / (M_PI * 3.25);
    float left = linearMotorVelocity;//+ w;
    float right = linearMotorVelocity;// - w;
    std::cout << v << "," << w << "," << eθ << std::endl;
    drive(left, right, 10);
  }
  drive_brake();
  Controller1.rumble(".");
}
void DriveToPoint3(float targetX, float targetY, float targetAngle,
             float accuracy = 1) {
  //θ - copy and paste theta
  float errorX = targetX - X;
  float errorY = targetY - Y;
  float 
  vp=15,
  vi=1,
  vd=3;
  float 
  wp=5,
  wi=0.5,
  wd=1;
  float ySum=0;
  float xSum=0;
  float ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
  float ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
  while (!((fabs(errorX) < accuracy) && (fabs(errorY)) < accuracy)) {
    errorX = targetX - X;
    errorY = targetY - Y;


    float prevX =ex;
    float prevY =ey;

    ey =  cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
    ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;

    float v = vp*ey + vi*ySum + vd*(prevY-ey);
    float w = wp*ex + wi*xSum + wd*(prevX-ex);
    
    float left = v + w;
    float right = v - w;

    ySum += ey;
    xSum += ex;
    drive(left, right, 10);
  }
  drive_brake();
             }