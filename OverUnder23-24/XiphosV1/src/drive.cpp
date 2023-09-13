#include "vex.h"
#include <vector>

using namespace vex;
// float TrackWidth = 8.5;
float C = M_PI * 3.25;
// extern double X, Y;
void terminalPrint(float a = 0, float b = 0, float c = 0, float d = 0, float e = 0)
{
  std::cout << a << "," << b << "," << c << "," << d << "," << e << std::endl;
}
double driveCurve(double x){
  return pow(x,2)/100 * (x/fabs(x));
  //return pow(M_E,(fabs(x)-100)/100)*x;
}
void drive(int lSpeed, int rSpeed, double wt)
{
  LF.spin(forward, lSpeed, pct);
  RF.spin(forward, rSpeed, pct);
  LB.spin(forward, lSpeed, pct);
  RB.spin(forward, rSpeed, pct);
  LM.spin(forward, lSpeed, pct);
  RM.spin(forward, rSpeed, pct);
  wait(wt, msec);
}
/**
 * @brief drives the robot based on voltage
 * @param lSpeed left side speed in percent
 * @param rSpeed right side speed in percent
 */
void voltDrive(double lSpeed, double rSpeed, double wt)
{

  LF.spin(forward, lSpeed * 120, voltageUnits::mV);
  LM.spin(forward, lSpeed * 120, voltageUnits::mV);
  LB.spin(forward, lSpeed * 120, voltageUnits::mV);

  RF.spin(forward, rSpeed * 120, voltageUnits::mV);
  RM.spin(forward, rSpeed * 120, voltageUnits::mV);
  RB.spin(forward, rSpeed * 120, voltageUnits::mV);
  wait(wt, msec);
}

void ffDrive(double lSpeed, double rSpeed, double wt)
{
  float kp = 0.25;
  float lfSpeed = lSpeed + (kp * (lSpeed - LF.velocity(pct)));
  float rfSpeed = rSpeed + (kp * (rSpeed - RF.velocity(pct)));
  float lbSpeed = lSpeed + (kp * (lSpeed - LB.velocity(pct)));
  float rbSpeed = rSpeed + (kp * (rSpeed - RB.velocity(pct)));
  LF.spin(forward, lfSpeed * 120, voltageUnits::mV);
  RF.spin(forward, rfSpeed * 120, voltageUnits::mV);
  LB.spin(forward, lbSpeed * 120, voltageUnits::mV);
  RB.spin(forward, rbSpeed * 120, voltageUnits::mV);
  wait(wt, msec);
}

/**
 * @brief brakes the robot based on brake type
 * @param Brake brake type
 */
void drive_brake(vex::brakeType Brake)
{
  LF.stop(Brake);
  RF.stop(Brake);
  LB.stop(Brake);
  LM.stop(Brake);
  RM.stop(Brake);
  RB.stop(Brake);
}
/*
!do not use
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
*/
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
    voltDrive(-lSpeed * speedMod, -rSpeed * speedMod, 10);
    sum += errorL;
    prevError = errorL;
  }
}
/**
 * @brief rotates the robot to a certain direction globally
 */
void rotate(double dir, double accuracy)
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
    voltDrive(speed / 3, -speed / 3, 10);

    wait(10, msec);
    prevError = error;
    sum = sum * 0.5 + error;
    error = dir - gyro1.rotation(degrees);
  }
  drive_brake();
}
//!no workie
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
//!no workie
//?te
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
 * // hvchfchf gnd
 */
//!no workie
void DriveToPoint3(float targetX, float targetY, float targetAngle,
                   float accuracy = 1)
{
  // θ - copy and paste theta
  float errorX = targetX - X;
  float errorY = targetY - Y;
  float
      vp = 15,
      vi = 0,
      vd = 3;
  float
      wp = .5,
      wi = 0.0,
      wd = 0;
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
    std::cout << ex << "," << ey << std::endl;
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

void driveAtAngle(float angle, float baseSpeed, float time)
{
  time = time * 1000;
  float kp = 6;
  float ki = 0.00;
  float kd = 30;
  float error = angle - gyro1.rotation();
  float prevError = error;
  float sum = 0;
  float speed;
  float vL, vR;
  float normFactor;
  float maxSpeed = 100;
  Brain.Timer.reset();
  Brain.Timer.clear();
  while (Brain.Timer.time() <= time)
  {
    error = angle - gyro1.rotation();
    speed = (kp * error) + (ki * sum) + (kd * (error - prevError));
    vL = baseSpeed + speed;
    vR = baseSpeed - speed;

    // calculate the normalization factor
    normFactor = maxSpeed / fmax(fabs(vL), fabs(vR));

    // clip speed if necessary
    if (normFactor < 1)
    {
      vL *= normFactor;
      vR *= normFactor;
    }

    // drive using voltDrive
    voltDrive(vL, vR, 10);
    

    if (fabs(error) / error != fabs(prevError) / prevError)
    {
      sum = 0;
    }
    else
    {
      sum += error;
    }
    std::cout <<Brain.Timer.time()<<","<< error << "," << speed << "," << kp * error << "," << ki * sum << "," << kd * (error - prevError) << std::endl; // pid tuning config
    wait(10,msec);
prevError = error;
  }
  drive_brake(coast);
}