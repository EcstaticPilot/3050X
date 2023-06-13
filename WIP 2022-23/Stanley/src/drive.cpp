#include "vex.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <bits/stdc++.h>
using namespace vex;

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

struct point
{
  float x;
  float y;
  point(float x, float y) : x(x), y(y) {}
};

/**
 * @breif distance between two points
 * @param x1 x coordinate of the first point
 * @param y1 y coordinate of the first point
 * @param x2 x coordinate of the second point
 * @param y2 y coordinate of the second point
 */
float distance2points(point p1, point p2)
{
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
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
float slope(std::vector<point> points, int point1, int point2)
{
  float x1 = points[point1].x;
  float y1 = points[point1].y;
  float x2 = points[point2].x;
  float y2 = points[point2].y;
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
/**
 * @brief finds the distance between the robot and a point
 * @param p1 struct of point containing the point
 */
float robotDistance(point p1)
{

  return (sqrt((X - p1.x) * (X - p1.x) + (Y - p1.y) * (Y - p1.y)));
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

/**
 * @brief if the next point is closer
 * @param initial the index of the current point
 * @param points an array containing the points
 */
int nextPoint(int initial, std::vector<point> points)
{
  if (robotDistance(points[initial].x, points[initial].y) < robotDistance(points[initial + 1].x, points[initial + 1].y))
  {
    return initial;
  }
  else
  {
    return initial + 1;
  }
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

/*
███████╗ ████████╗  █████╗  ███╗   ██╗ ██╗      ███████╗ ██╗   ██╗
██╔════╝ ╚══██╔══╝ ██╔══██╗ ████╗  ██║ ██║      ██╔════╝ ╚██╗ ██╔╝
███████╗    ██║    ███████║ ██╔██╗ ██║ ██║      █████╗    ╚████╔╝
╚════██║    ██║    ██╔══██║ ██║╚██╗██║ ██║      ██╔══╝     ╚██╔╝
███████║    ██║    ██║  ██║ ██║ ╚████║ ███████╗ ███████╗    ██║
╚══════╝    ╚═╝    ╚═╝  ╚═╝ ╚═╝  ╚═══╝ ╚══════╝ ╚══════╝    ╚═╝
*/




float lerp(float a, float b, float t)
{
  return ((1 - t) * a + t * b);
}
/**
 * @brief finds the point on a bezier curve
 * @param bezier an array containing the controll points
 * @param t the t value
 * @param XorY whether to return the x or y value. use lowercase x or y
 */
float bezier(float bezier[][2], float t, std::string XorY)
{
  int n;
  if (XorY == "x")
  {
    n = 0;
  }
  else if (XorY == "y")
  {
    n = 1;
  }
  float a1 = bezier[0][n];
  float a2 = bezier[1][n];
  float a3 = bezier[2][n];
  float a4 = bezier[3][n];
  //bernestein polynomials
  return (a1*(pow(-t,3)+3*pow(t,2)-3*t+1)+a2*(3*pow(t,3)-6*pow(t,2)+3*t)+a3*(-3*pow(t,3)+3*pow(t,2))+a4*pow(t,3));
  //nested lerps
  float lerp1 = lerp(a1, a2, t);
  float lerp2 = lerp(a2, a3, t);
  float lerp3 = lerp(a3, a4, t);
  float lerp4 = lerp(lerp1, lerp2, t);
  float lerp5 = lerp(lerp2, lerp3, t);
  float lerp6 = lerp(lerp4, lerp5, t);
  return lerp6;
}

/**
 * @brief finds the distance of a point from a line
 * @param points an array containing the points of the line
 * @param point1 the index first point of the line
 * @param point2 the index second point of the line
 */
float perpendicularDist(std::vector<point> points, int point1, int point2)
{
  point p1 = points[point1];
  point p2 = points[point2];
  point robot = point(X, Y);
  // calculate distance
  float A = p2.y - p1.y;
  float B = p1.x - p2.x;
  float C = p2.x * p1.y - p1.x * p2.y;
  float d = fabs((A * robot.x + B * robot.y + C)) / sqrt(A * A + B * B);
  // check for intersection
  float a = robotDistance(p1);
  float b = distance2points(p1,p2);
  float c = robotDistance(p2);
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
 * @brief finds the sign of the distance bettwen two points
 * @param points an array containing the points
 * @param point1 the first point
 * @param point2 the second point
 */
int signOfDistance(std::vector<point> points, int p1, int p2)
{
  float x1 = points[p1].x;
  float y1 = points[p1].y;
  float x2 = points[p2].x;
  float y2 = points[p2].y;
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
 * @brief stanley controller for following a path of points
 * @param points an array containing points to follow
 * @param length the length of the array
 */
void stanley(float points[][2], int length)
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

  int num_col = 2;

  // Number of rows
  int num_row = 10;
  // int i = 0;
  // Initializing the 2-D vector
  std::vector<point> Bpoints(num_row, point(0, 0));
  int tval = 0;
  // fil the 2d vector with the points
  for (int i = 0; i < 20; i++)
  {

    Bpoints[i].x= bezier(points, tval, "x");
    Bpoints[i].y= bezier(points, tval, "y");
    tval += 0.005;
  }
  // print out the array to the terminal

  Brain.Screen.print("stanley");

  int pointClosest = 0;
  Brain.Screen.drawRectangle(0, 0, 480, 240, yellow);
  while (true)
  {
    // find the closest point
    // pointClosest =closestPoint(points); //alternate way to find closest point
    pointClosest = nextPoint(pointClosest, Bpoints);
    Brain.Screen.print("0.5");
    if (!(pointClosest == 1))
    {
      //if the closest point is not the second point, erase all prevoius values until it is
      Bpoints.erase(Bpoints.begin(), Bpoints.begin() + pointClosest - 1);
      //fill the vecotr until the length is 20
      while (Bpoints.size() < 20)
      {
        Bpoints.push_back(point(bezier(points, tval, "x"), bezier(points, tval, "y")));
        tval += 0.005;
      }
    }

    // find the distance to the line segment before and after the closest point

    float segment1dist = (!(pointClosest == 0) ? (perpendicularDist(Bpoints, (pointClosest - 1), pointClosest)) : (900000000000)); // distance of the path segment before the closest point

    float segment2dist = perpendicularDist(Bpoints, pointClosest, pointClosest + 1); // distance of the path segment after the closest point
    // if neither one has an intersection, find the distance to the line segment after the next point

    if (segment1dist > 300000 && segment2dist > 300000)
    {
      segment2dist -= 300000;
    }

    // find the distance to the closest line segment and the heading of that line segment
    if (segment2dist < segment1dist)
    {
      pathDistance = segment2dist;
      pathHeading = slope(Bpoints, pointClosest, pointClosest + 1); //
      sign = signOfDistance(Bpoints, pointClosest, pointClosest + 1);
    }
    else
    {
      pathDistance = segment1dist;
      pathHeading = slope(Bpoints, pointClosest - 1, pointClosest);
      sign = signOfDistance(Bpoints, pointClosest - 1, pointClosest);
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
    std::cout << X << ",,," << Y << std::endl;
    vex::wait(20, msec);
  }
  drive(0, 0, 10);
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
  drive_brake;
  std::cout << "done" << std::endl;
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
}

/*
██████╗  ██╗   ██╗ ██████╗  ███████╗     ██████╗  ██╗   ██╗ ██████╗  ███████╗ ██╗   ██╗ ██╗ ████████╗
██╔══██╗ ██║   ██║ ██╔══██╗ ██╔════╝     ██╔══██╗ ██║   ██║ ██╔══██╗ ██╔════╝ ██║   ██║ ██║ ╚══██╔══╝
██████╔╝ ██║   ██║ ██████╔╝ █████╗       ██████╔╝ ██║   ██║ ██████╔╝ ███████╗ ██║   ██║ ██║    ██║
██╔═══╝  ██║   ██║ ██╔══██╗ ██╔══╝       ██╔═══╝  ██║   ██║ ██╔══██╗ ╚════██║ ██║   ██║ ██║    ██║
██║      ╚██████╔╝ ██║  ██║ ███████╗     ██║      ╚██████╔╝ ██║  ██║ ███████║ ╚██████╔╝ ██║    ██║
╚═╝       ╚═════╝  ╚═╝  ╚═╝ ╚══════╝     ╚═╝       ╚═════╝  ╚═╝  ╚═╝ ╚══════╝  ╚═════╝  ╚═╝    ╚═╝
*/

/**
 * @brief drive to a point with a curve
 * @param x x coordinate of the point
 * @param y y coordinate of the point
 * @param speed speed to drive at
 */
void curveDrive(double x, double y, double speed)
{
  double dist = robotDistance(x, y);
  double errorX = x - X;
  double errorY = y - Y;
  double localX = errorX * cos(gyro1.rotation()) - errorY * sin(gyro1.rotation());
  double curvature = 2 * localX / (dist * dist);
  float L = speed * (2 + (curvature * TrackWidth)) / 2;
  float R = speed * (2 - (curvature * TrackWidth)) / 2;
  drive(L, R, 10);
}
/**
 * @brief fidn the intersectionof a line and a circle
 * @param points an array containing the points
 * @param lineSegment the index of the start of the line segment
 * @param ld the distance from the robot to the point
 * @param xOrY whether to return the x or y coordinate of the intersection
 */
float lineCircleIntersection(float points[][2], int lineSegment, float ld)
{
  // convert the x and y coordinates of the line segment into the form y=mx+b

  float x1 = points[lineSegment][0];
  float y1 = points[lineSegment][1];
  float x2 = points[lineSegment + 1][0];
  float y2 = points[lineSegment + 1][1];

  float h = X;
  float k = Y;
  float r = ld;
  // Translate the line segment and circle so that the circle is centered at the origin
  double x1p = x1 - h;
  double y1p = y1 - k;
  double x2p = x2 - h;
  double y2p = y2 - k;

  // Calculate the slope and y-intercept of the translated line segment
  double m = (y2p - y1p) / (x2p - x1p);
  double b = y1p - m * x1p;

  // Calculate the discriminant
  double discriminant = pow((2 * m * b), 2) - 4 * ((pow(m, 2) + 1) * (pow(b, 2) - pow(r, 2)));

  if (discriminant > 0)
  {
    // Two distinct intersection points
    double x_1 = (-2 * m * b + sqrt(discriminant)) / (2 * (pow(m, 2) + 1));
    double y_1 = m * x_1 + b;
    float tval1 = (x_1 - x1) / (x2 - x1) + lineSegment;

    double x_2 = (-2 * m * b - sqrt(discriminant)) / (2 * (pow(m, 2) + 1));
    double y_2 = m * x_2 + b;
    float tval2 = (x_2 - x1) / (x2 - x1) + lineSegment;
    float tval3 = fmax(tval1, tval2);

    // make sure to check if both intersection points are on the line segment
    if (tval3 > 0 && tval3 < 1)
    {
      return tval3 + lineSegment;
    }
    else
    {
      return -1;
    }
  }
  else if (discriminant == 0)
  {
    // Tangent intersection point
    double x_1 = -2 * m * b / (2 * (pow(m, 2) + 1));
    double y_1 = m * x_1 + b;

    float tval1 = (x_1 - x1) / (x2 - x1);
    // check if the intersection is on the line segment
    if (tval1 > 0 && tval1 < 1)
    {
      return tval1 + lineSegment;
    }
    else
    {
      return -1;
    }
  }
  else
  {
    // No intersection
    return -1;
  }
}

/**
 * @brief pure pursuit algorithm
 * @param points an array containing the points
 * @param length the length of the array
 */
void purePursuit(float points[][2], int length)
{
  int pointLookahead = 0;
  float ld = 12;
  int pointClosest = 0;
  float tval;
  float tvali;
  float prevTval = -1;
  float x1, y1;
  while (true)
  {
    pointClosest = closestPoint(points);
    if (length == pointClosest - 1)
    {
      break;
    }
    // find the lookahead point
    for (int i = 0; i < length; i++)
    {
      tvali = lineCircleIntersection(points, i, ld);
      if (tvali > tval)
      {
        pointLookahead = i;
        tval = tvali;
      }
    }
    // std::cout<<tval<<","<<pointLookahead<<std::endl;
    tval = tval - pointLookahead;
    if (tval < 0)
    {
      x1 = points[pointClosest][0];
      y1 = points[pointClosest][1];
    }
    else
    {
      x1 = points[pointLookahead][0] + tval * (points[pointLookahead + 1][0] - points[pointLookahead][0]);
      y1 = points[pointLookahead][1] + tval * (points[pointLookahead + 1][1] - points[pointLookahead][1]);
    }
    std::cout << x1 << "," << y1 << std::endl;
    curveDrive(x1, y1, 25);
  }
}
