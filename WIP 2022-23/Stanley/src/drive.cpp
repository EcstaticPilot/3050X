#include "vex.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
float TrackWidth = 15;
float C = M_PI * 3.25;
extern double X, Y;

void drive(int lSpeed, int rSpeed, double wt)
{
  LF.spin(forward, lSpeed, pct);
  RF.spin(forward, rSpeed, pct);
  LB.spin(forward, lSpeed, pct);
  RB.spin(forward, rSpeed, pct);
  wait(wt, msec);
}

/**
 * @brief brakes the robot based on brake type
*/
void drive_brake(vex::brakeType Brake = brake)
{
  LF.stop(Brake);
  RF.stop(Brake);
  LB.stop(Brake);
  RB.stop(Brake);
}

void forward_dist(float dist)
{
  // dist is in inches
  // 36.15 for 1 wheel rotation a second for a gear ratio of 3/5
  float t_to_run = fabs(dist) / C;
  LF.spin(forward, (dist / fabs(dist)) * 36.15, rpm);
  RF.spin(forward, (dist / fabs(dist)) * 36.15, rpm);
  LB.spin(forward, (dist / fabs(dist)) * 36.15, rpm);
  RB.spin(forward, (dist / fabs(dist)) * 36.15, rpm);
  wait(t_to_run, sec);
  drive_brake();
}
/**
 * @brief drives the robot a certain distance
 * @param dist distance in inches
 * @param speedMod speed modifier
 * @param accuracy accuracy in inches
*/
void inchDrive(double target, double speedMod = 1)
{
  double kp = 1, ki = 0, kd = 0;
  double bStart = RotationB.position(turns) * 360;
  double lStart = (RotationL.position(turns) * 360 /*get degrees*/) * (M_PI / 180 /*do radians*/) * 1.3926 /*multiply by radius*/;
  double errorL = target - lStart;
  double errorB = 0 - bStart;
  double prevError = errorL;
  double sum = 0;
  double accuracy = 1;
  while (fabs(errorL) > accuracy)
  {
    errorL = (target + lStart) - (RotationL.position(turns) * 360 /*get degrees*/) * (M_PI / 180 /*do radians*/) * 1.3926 /*multiply by radius*/;
    errorB = 0 - bStart;
    double lSpeed = (kp * errorL) + (ki * sum) + (kd * (prevError - errorL)) + 0.2 * errorB;
    double rSpeed = (kp * errorL) + (ki * sum) + (kd * (prevError - errorL)) - 0.2 * errorB;
    ;
    drive(-lSpeed * speedMod, -rSpeed * speedMod, 10);
    sum += errorL;
    prevError = errorL;
  }
}
/**
 * @brief rotates the robot to a certain direction globally
*/
void rotate(double dir, double accuracy = 1)
{
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
  while (fabs(error) > accuracy)
  {
    speed = (Kp * error) + (Ki * sum) + (Kd * (error - prevError));
    drive(speed / 3, -speed / 3, 10);

    wait(10, msec);
    prevError = error;
    sum = sum * 0.5 + error;
    error = dir - gyro1.rotation(degrees);
  }
  drive_brake();
}

void DriveToPoint(double targetX, double targetY, float speedMult = 1)
{
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

  while ((fabs(targetX - X) > accuracy) || (fabs(targetY - Y) > accuracy))
  {

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
    if (speedMult < 0)
    {
      double idk;
      idk = rSpeed;
      rSpeed = lSpeed;
      lSpeed = idk;
    }
    std::cout << lSpeed << "," << rSpeed << "," << dError << "," << tError << std::endl;
    drive((lSpeed)*speedMult, (rSpeed)*speedMult, 12);
    dErrorOld = dError;
    dSum += dError;

    tErrorOld = tError;
    tSum += tError;
  }
  drive_brake();
}

void DriveToPoint2(float targetX, float targetY)
{

  rotate((atan2(targetY - Y, X - targetX) * 180 / M_PI));
  std::cout << gyro1.rotation() << std::endl;
  wait(1, sec);
  // inchDrive(-(sqrt((targetX - X) * (targetX -  X) + (targetY - Y) * (targetY - Y))));
  std::cout << X << "," << Y << std::endl;
}
/**
 * @brief Ramsete control algorithm
*/
void RAMSETE(float targetX, float targetY, float targetAngle,
             float accuracy = 1)
{
  // https://wiki.purduesigbots.com/software/control-algorithms/ramsete
  // θ - copy and paste theta
  float errorX = targetX - X;
  float errorY = targetY - Y;
  float smallScalar = 0.01;
  float beta = 0.3;
  float zeta = .2;
  while (!((fabs(errorX) < accuracy) && (fabs(errorY)) < accuracy))
  {
    errorX = targetX - X;
    errorY = targetY - Y;
    float errorθ = -gyro1.rotation() - (atan2(targetY - Y, X - targetX) * 180 / M_PI);
    float ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
    float ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
    float eθ = errorθ;

    float vd = errorY * smallScalar;
    float wd = eθ * smallScalar;
    float k = 2 * zeta * sqrt(pow(wd, 2) + beta * pow(vd, 2));
    float v = (vd * cos(eθ)) + (k * ey);
    float w = wd + k * eθ + (beta * vd * sin(eθ) * ex) / eθ;

    float linearMotorVelocity = v / (M_PI * 3.25);
    float left = linearMotorVelocity;  //+ w;
    float right = linearMotorVelocity; // - w;
    std::cout << v << "," << w << "," << eθ << std::endl;
    drive(left, right, 10);
  }
  drive_brake();
  Controller1.rumble(".");
}

/**
 * @brief Drives to a point using the gyro
*/
void DriveToPoint3(float targetX, float targetY, float targetAngle,
                   float accuracy = 1)
{
  // θ - copy and paste theta
  float errorX = targetX - X;
  float errorY = targetY - Y;
  float
      vp = 15,
      vi = 1,
      vd = 3;
  float
      wp = 5,
      wi = 0.5,
      wd = 1;
  float ySum = 0;
  float xSum = 0;
  float ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
  float ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
  while (!((fabs(errorX) < accuracy) && (fabs(errorY)) < accuracy))
  {
    errorX = targetX - X;
    errorY = targetY - Y;

    float prevX = ex;
    float prevY = ey;

    ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
    ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;

    float v = vp * ey + vi * ySum + vd * (prevY - ey);
    float w = wp * ex + wi * xSum + wd * (prevX - ex);

    float left = v + w;
    float right = v - w;

    ySum += ey;
    xSum += ex;
    drive(left, right, 10);
  }
  drive_brake();
}
/**
 * @brief drive with a curve
 * @param targetVel target velocity for the robot to drive at
 * @param curvature how mcuh the robot should curve
 */
void curveDrive(float targetVel, float curvature)
{
  // calculate speeds
  float turnSpeed = 0.5 * targetVel * curvature * TrackWidth;
  float vL = targetVel + turnSpeed;
  float vR = targetVel - turnSpeed;

  // normalize
  float normFactor = 100 / fmax(fabs(vL), fabs(vR));

  // clip speed if necessary
  if (normFactor < 1)
  {
    vL *= normFactor;
    vR *= normFactor;
  }

  // drive
  drive(vL, vR, 10);
}
/**
 * @brief convert radians to degrees
 * @param rad radians(double)
 */
double RadToDeg(double rad)
{
  return (rad * 180 / M_PI);
}
/**
 * @brief convert degrees to radians
 * @param deg degrees (double)
 */
double DegToRad(double deg)
{
  return (deg * M_PI / 180);
}
/**
 * @brief finds the distance between the robot and a point
 * @param x x coordinate of the point
 * @param y y coordinate of the point
 */
double robotDistance(double x, double y)
{

  return ( sqrt(pow(X - x, 2) + pow(Y - y, 2)) );
}
/**
 * @brief finds the closest point to the robot
 * @param points an array containing the points
 */
int closestPoint(double points[][2])
{
  int npoints = sizeof points / sizeof points[2];
  int closest = 0;

  for (int i = 0; i < npoints; i++)
  {
    if(robotDistance(points[closest][1],points[closest][2])<(points[i][1],points[i][2])){
      closest=i;
    }
  }
  return(closest);
}
/**
 * @brief finds the distance of a point from a line
 * @param points an array containing the points of the line
 * @param point1 the index first point of the line
 * @param point2 the index second point of the line
 */
float perpendicularDist(double points[][2],int point1,int point2)
{
  double x1 = points[point1][0];
  double y1 = points[point1][1];
  double x2 = points[point2][0];
  double y2 = points[point2][1];
  double x3 = X;
  double y3 = Y;
  double d = fabs((y2 - y1) * x3 - (x2 - x1) * y3 + x2 * y1 - y2 * x1) / sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
  return d;
}

/**
 * @brief stanley controller for following a path of points
 * @param points an array containing points to follow
 */
void stanley(double points[][2])
{

  double ld;
  double v = 50;
  double kv = 0.25;
  double pathDistance;
  double pathHeading;
  double ldAngle;
  double pathError;
  while (1)
  {
    closestPoint(points);
    ld = kv / v;
    ldAngle = RadToDeg(atan2(pathDistance, ld)) - gyro1.yaw();
    pathError = pathHeading - gyro1.yaw();
    curveDrive(50, ldAngle + pathError);
    v = LF.velocity(pct) + RF.velocity(pct) / 2;
  }
}








