#include "vex.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
float TrackWidth = 17 / 2;
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
 * @breif distance between two points
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 */
float distance2points(float x1, float y1, float x2, float y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/**
 * @brief convert radians to degrees
 * @param rad radians(float)
 */
float RadToDeg(float rad)
{
  return (rad * 180 / M_PI);
}

/**
 * @brief convert degrees to radians
 * @param deg degrees (float)
 */
float DegToRad(float deg)
{
  return (deg * M_PI / 180);
}

bool zeroCrossing(float a, float b)
{
  int A = fabs(a) / a;
  int B = fabs(b) / b;
  if (A != B)
  {
    return true;
  }
  else
  {
    return false;
  }
}
// a class containing the x and y coordinates of a point

/**
 * @brief finds the slope between two points
 * @param points an array containing the points
 * @param point1 the first point
 * @param point2 the second point
 */
float slope(float points[][2], int point1, int point2)
{
  float x1 = points[point1][0];
  float y1 = points[point1][1];
  float x2 = points[point2][0];
  float y2 = points[point2][1];
  float slope = RadToDeg(atan2(x2 - x1, y2 - y1));
  return slope;
}

/**
 * @brief finds the distance between the robot and a point
 * @param x x coordinate of the point
 * @param y y coordinate of the point
 */
float robotDistance(float x, float y)
{
  
  return (sqrt((X - x) * (X - x) + (Y - y) * (Y - y)));
}

int signOfDistance(float points[][2], int point1, int point2)
{
  float x1 = points[point1][0];
  float y1 = points[point1][1];
  float x2 = points[point2][0];
  float y2 = points[point2][1];
  float x3 = X;
  float y3 = Y;
  float d = (x2 - x1) * (y1 - y3) - (x1 - x3) * (y2 - y1);
  if (d > 0)
  {
    return -1;
  }
  else if (d < 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/**
 * @brief drive to a point with a curve
 * @param x x coordinate of the point
 * @param y y coordinate of the point
 * @param speed speed to drive at
 */
void curveDrive2(double x, double y, double speed)
{
  double L = robotDistance(x, y);
  double errorX = x - X;
  double errorY = y - Y;
  double localX = errorX * cos(gyro1.rotation()) - errorY * sin(gyro1.rotation());
  double curvature = 2 * localX / (L * L);
  float L = speed*(2+(curvature*TrackWidth))/2;
  float R = speed*(2-(curvature*TrackWidth))/2;
  drive(L, R, 10);
}
void lineCircleIntersection(float points[][2],int lineSegment, float ld){
//convert the x and y coordinates of the line segment into the form y=mx+b
float x1=points[lineSegment][0];
float y1=points[lineSegment][1];
float x2=points[lineSegment+1][0];
float y2=points[lineSegment+1][1];
float m=(y2-y1)/(x2-x1);
float b=y1-m*x1;
//find the equation of the circle in form (x-h)^2+(y-k)^2=r^2
float h=X;
float k=Y;
float r=ld;
//find the discriminant
float discriminant=pow((2 * m * b - 2 * h),2) - 4 * ( (m*m) + 1) * ((b*b) - (r*r) + (h*h) - 2 * k * b + (k*k));
 






}
/**
 * @brief finds the closest point to the robot
 * @param points an array containing the points
 */
int closestPoint(float points[][2])
{
  double min = robotDistance(points[0][0], points[0][1]);
  int index = 0;

  for (int i = 0; i < sizeof points; i++)
  {
    float dist = robotDistance(points[i][0], points[i][1]);
    if (dist < min)
    {
      min = dist;
      index = i;
    }
  }
  return index;
}

int nextPoint(int initial, float points[][2])
{
  if (robotDistance(points[initial][0], points[initial][1]) < robotDistance(points[initial + 1][0], points[initial + 1][1]))
  {
    return initial;
  }
  else
  {
    return initial + 1;
  }
}
/**
 * @brief finds the distance of a point from a line
 * @param points an array containing the points of the line
 * @param point1 the index first point of the line
 * @param point2 the index second point of the line
 */
float perpendicularDist(float points[][2], int point1, int point2)
{
  float x1 = points[point1][0];
  float y1 = points[point1][1];
  float x2 = points[point2][0];
  float y2 = points[point2][1];
  float x3 = X;
  float y3 = Y;
  // calculate distance
  float A = y2 - y1;
  float B = x1 - x2;
  float C = x2 * y1 - x1 * y2;
  float d = fabs((A * x3 + B * y3 + C)) / sqrt(A * A + B * B);
  // check for intersection
  float a = robotDistance(x1, y1);
  float b = distance2points(x1, y1, x2, y2);
  float c = robotDistance(x2, y2);
  // check if the robot is within the line using law of cosines
  float angle1 = RadToDeg(acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b)));
  float angle2 = RadToDeg(acos((pow(c, 2) + pow(b, 2) - pow(a, 2)) / (2 * c * b)));
  // if the robot is not within the line, add a large number to the distance
  if (angle1 > 90 || angle2 > 90)
  {
    d += 300000;
  }
  return d;
}

/**
 * @brief stanley controller for following a path of points
 * @param points an array containing points to follow
 */
void stanley(float points[][2],int length)
{
  float kp = 0.5;
  float ki = 0.005;
  float kd = 0.00;
  float ld;
  float v;
  float kv = 1;
  float pathDistance;
  float pathHeading;
  float ldAngle;
  float prevError = 0;
  float totalError = 0;
  int sign;
  Brain.Screen.print("stanley");
  int i = 0;
  int pointClosest = 0;
  Brain.Screen.drawRectangle(0, 0, 480, 240, yellow);
  while (true)
  {
    i++;

    Brain.Screen.print("%.1f", i);
    // find the closest point
    // pointClosest =closestPoint(points); //alternate way to find closest point
    pointClosest = nextPoint(pointClosest, points);
    Brain.Screen.print("0.5");
    // if the closest point is the last point, break from the while loop using the sizeof function
    if (pointClosest == length - 1)
    {
      break;
    }

    // find the distance to the line segment before and after the closest point

    float segment1dist = (!(pointClosest == 0) ? (perpendicularDist(points, (pointClosest - 1), pointClosest)) : (900000000000)); // distance of the path segment before the closest point

    float segment2dist = perpendicularDist(points, pointClosest, pointClosest + 1); // distance of the path segment after the closest point
    // if neither one has an intersection, find the distance to the line segment after the next point

    if (segment1dist > 300000 && segment2dist > 300000)
    {
      segment2dist -= 300000;
    }

    // find the distance to the closest line segment and the heading of that line segment
    if (segment2dist < segment1dist)
    {
      pathDistance = segment2dist;
      pathHeading = slope(points, pointClosest, pointClosest + 1); //
      sign = signOfDistance(points, pointClosest, pointClosest + 1);
    }
    else
    {
      pathDistance = segment1dist;
      pathHeading = slope(points, pointClosest - 1, pointClosest);
      sign = signOfDistance(points, pointClosest - 1, pointClosest);
    }
    // calculate lookahead distance
    v = LF.velocity(pct) + RF.velocity(pct) / 2;
    ld = 5; // v / kv;

    // calculate the angle to the global angle to the lookahead point
    ldAngle = RadToDeg(atan2(pathDistance, ld));

    // calculate the error
    float error = (ldAngle * sign + pathHeading - gyro1.yaw(deg));

    // calculate the PID output
    float output = (error * kp) + (totalError)*ki + (prevError - error) * kd;

    

    // drive
    drive(25 + output, 25 - output, 10);

    // reset integral if error crosses zero
    if (zeroCrossing(error, prevError))
    {
      totalError = 0;
    }
    else
    {
      totalError += error;
    }
    // prevoius error
    prevError = error;
    //  update speed

    // print the values
    // pathDistance<<","<<ldAngle*-sign<<","<<perpendicularDist(points,0,1)<<","
    //  std::cout<<"x="<<X<<", y= "<<Y<<", pointclosest= "<<pointClosest<<" error= "<<error<<std::endl
    //  <<"seg1dist= "<<segment1dist<<", seg2dist= "<<segment2dist<<", pathdist= "<<pathDistance<<std::endl<<std::endl;
    std::cout << X << ",,," << Y<< std::endl;
    wait(10, msec);
  }
  drive(0, 0, 10);
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
  drive_brake;
  std::cout << "done" << std::endl;
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
}

void purePursuit(float points[][2],int length){
  float ld=12;
  int pointClosest=0;
  while(true){
    if(length==pointClosest-1){
      break;
    }
    //find the lookahead point






  }



}




