#include "vex.h"
#include <iostream>
#include <stdio.h>

#include <math.h>

float C = M_PI * 3.25;
extern double X,Y;

void drive(int lSpeed, int rSpeed, double wt) {
  LF.spin(forward, lSpeed, pct);
  RF.spin(forward, rSpeed, pct);
  LB.spin(forward, lSpeed, pct);
  RB.spin(forward, rSpeed, pct);
  wait(wt, msec);
}

void drive_brake(vex::brakeType Brake=brake) {
  LF.stop(Brake);
  RF.stop(Brake);
  LB.stop(Brake);
  RB.stop(Brake);
}

void forward_dist(float dist) {
  // dist is in inches
  // 36.15 for 1 wheel rotation a second for a gear ratio of 3/5
  float t_to_run = dist/C;
  LF.spin(forward, 36.15, rpm);
  RF.spin(forward, 36.15, rpm);
  LB.spin(forward, 36.15, rpm);
  RB.spin(forward, 36.15, rpm);
  wait(t_to_run, sec);
  drive_brake();
}

void inchDrive(double target, double speedMod=1){
  double kp=1,ki=0,kd=0;
  double bStart=RotationB.position(turns)*360;
  double lStart=(RotationL.position(turns)*360/*get degrees*/)*(M_PI/180/*do radians*/)*1.3926/*multiply by radius*/;
  double errorL=target-lStart;
  double errorB=0-bStart;
  double prevError=errorL;
  double sum=0;
  double accuracy=1;
  while(fabs(errorL)>accuracy){
  errorL=(target+lStart)-  (RotationL.position(turns)*360/*get degrees*/)*(M_PI/180/*do radians*/)*1.3926/*multiply by radius*/;
  errorB=0-bStart; 
  double lSpeed=(kp*errorL)+(ki*sum)+(kd*(prevError-errorL))+0.2*errorB;
  double rSpeed=(kp*errorL)+(ki*sum)+(kd*(prevError-errorL))-0.2*errorB;;
  drive(-lSpeed*speedMod, -rSpeed*speedMod, 10);
  sum+=errorL;
  prevError=errorL;
  }
}


void rotate(double dir, double accuracy = 1) {
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
    drive(speed/3, -speed/3, 10);

    wait(10, msec);
    prevError = error;
    sum = sum * 0.5 + error;
    error = dir - gyro1.rotation(degrees);
  }
  drive_brake();
}

/* void forward_drive(float dist) {

  float start_position;
  float curr_position;
  float error;
  float old_error;
  float sum;

  wait(100, msec);

  Brain.Screen.printAt(100, 150, "hello world");
  Brain.Screen.printAt(10, 110, "first: %f,%f ", X, Y);
  forward_dist(1);
  wait(100, msec);
  Brain.Screen.printAt(10, 120, "second: %f,%f ", X, Y);

  
  drive_brake();
} */

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
  //inchDrive(-(sqrt((targetX - X) * (targetX -  X) + (targetY - Y) * (targetY - Y))));
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