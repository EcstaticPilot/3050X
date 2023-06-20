#include "vex.h"
#include <vector>

using namespace vex;

float TrackWidth = 8.5;
float C = M_PI * 3.25;
//extern double X, Y;
void terminalPrint(float a = 0, float b = 0, float c = 0, float d = 0, float e = 0)
{
  std::cout << a << "," << b << "," << c << "," << d << "," << e << std::endl;
}
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
 * @param Brake brake type
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

// start off path stuff

struct point
{
  float x;
  float y;
  point(float x, float y) : x(x), y(y) {}
};

struct bezierPoint
{
  float x;
  float y;
  float tval;
  float angle;
  float curvature;
  bezierPoint(float x, float y, float tval, float angle, float curvature) : x(x), y(y), tval(tval), angle(angle), curvature(curvature) {}
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
  return hypot(p1.x - p2.x, p1.y - p2.y);
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

float slope(point p1, point p2)
{
  float x1 = p1.x;
  float y1 = p1.y;
  float x2 = p2.x;
  float y2 = p2.y;
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

  return (hypot(X - x, Y - y));
}
/**
 * @brief finds the distance between the robot and a point
 * @param p1 struct of point containing the point
 */
float robotDistance(point p1)
{

  return (hypot(X - p1.x, Y - p1.y));
}

/**
 * @brief finds the closest point to the robot
 * @param points an array containing the points
 */
int closestPoint(float points[][2], int size)
{
  double min = robotDistance(points[0][0], points[0][1]);
  int index = 0;

  for (int i = 0; i < size; i++)
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
 * @brief finds the closest point to the robot
 * @param points an array containing the points as a struct
 */
int closestPoint(std::vector<point> points, int size)
{
  double min = robotDistance(points[0].x, points[0].y);
  int index = 0;

  for (int i = 0; i < size; i++)
  {
    float dist = robotDistance(points[i].x, points[i].y);
    if (dist < min)
    {
      min = dist;
      index = i;
    }
  }
  return index;
}

/**
 * @brief finds the closest point to the robot
 * @param points an array containing the points as a struct
 */
int closestPoint(std::vector<bezierPoint> points, int size)
{
  double min = robotDistance(points[0].x, points[0].y);
  int index = 0;

  for (int i = 0; i < size; i++)
  {
    float dist = robotDistance(points[i].x, points[i].y);
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
 * @brief finds the point on a bezier curve and reutrns a point struct
 * @param bezier an array containing the control points
 * @param t the t value
 */
point bezier(float bezierPoints[][2], float t)
{
  // determines whcih bezier curve to find when working with splines
  int n = floor(t);
  float x1 = bezierPoints[0 + 4 * n][0];
  float x2 = bezierPoints[1 + 4 * n][0];
  float x3 = bezierPoints[2 + 4 * n][0];
  float x4 = bezierPoints[3 + 4 * n][0];

  float y1 = bezierPoints[0 + 4 * n][1];
  float y2 = bezierPoints[1 + 4 * n][1];
  float y3 = bezierPoints[2 + 4 * n][1];
  float y4 = bezierPoints[3 + 4 * n][1];
  // bernestein polynomials
  // subtracts the n value from t to get the t value for the bezier curve
  t -= n;
  point bezierPoint(
      /*x points*/ (x1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + x2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) + x3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + x4 * pow(t, 3)),
      /*y points*/ (y1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + y2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) + y3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + y4 * pow(t, 3)));
  return (bezierPoint);
}

point derivativeBezier(float bezierPoints[][2], float t)
{

  int n = floor(t);
  float x1 = bezierPoints[0 + 4 * n][0];
  float x2 = bezierPoints[1 + 4 * n][0];
  float x3 = bezierPoints[2 + 4 * n][0];
  float x4 = bezierPoints[3 + 4 * n][0];

  float y1 = bezierPoints[0 + 4 * n][1];
  float y2 = bezierPoints[1 + 4 * n][1];
  float y3 = bezierPoints[2 + 4 * n][1];
  float y4 = bezierPoints[3 + 4 * n][1];
  // bernestein polynomials
  // subtracts the n value from t to get the t value for the bezier curve
  t -= n;
  point bezierPoint(
      /*x points*/ (x1 * (-3 * pow(t, 2) + 6 * t - 3) + x2 * (9 * pow(t, 2) - 12 * t + 3) + x3 * (-9 * pow(t, 2) + 6 * t) + x4 * 3 * pow(t, 2)),
      /*y points*/ (y1 * (-3 * pow(t, 2) + 6 * t - 3) + y2 * (9 * pow(t, 2) - 12 * t + 3) + y3 * (-9 * pow(t, 2) + 6 * t) + y4 * 3 * pow(t, 2)));
  return (bezierPoint);
}

float curvature(float bezierPoints[][2], float t)
{
  int n = floor(t);
  float x0 = bezierPoints[0 + 4 * n][0];
  float x1 = bezierPoints[1 + 4 * n][0];
  float x2 = bezierPoints[2 + 4 * n][0];
  float x3 = bezierPoints[3 + 4 * n][0];

  float y0 = bezierPoints[0 + 4 * n][1];
  float y1 = bezierPoints[1 + 4 * n][1];
  float y2 = bezierPoints[2 + 4 * n][1];
  float y3 = bezierPoints[3 + 4 * n][1];
  // bernestein polynomials
  // subtracts the n value from t to get the t value for the bezier curve
  t -= n;
  float dt = 1 - t;

  // Calculate the components of the first and second derivatives
  float dx_dt = 3 * dt * dt * (x1 - x0) + 6 * dt * t * (x2 - x1) + 3 * t * t * (x3 - x2);
  float dy_dt = 3 * dt * dt * (y1 - y0) + 6 * dt * t * (y2 - y1) + 3 * t * t * (y3 - y2);
  float d2x_dt2 = 6 * dt * (x2 - 2 * x1 + x0) + 6 * t * (x3 - 2 * x2 + x1);
  float d2y_dt2 = 6 * dt * (y2 - 2 * y1 + y0) + 6 * t * (y3 - 2 * y2 + y1);

  // Calculate the curvature
  float numerator = fabs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2);
  float denominator = pow(dx_dt * dx_dt + dy_dt * dy_dt, 1.5);
  float curvature = numerator / denominator;
  return curvature;
}

/**
 * @brief finds the point on a bezier curve and reutrns a bezier point struct
 * @param bezier an array containing the control points
 * @param t the t value
 */
bezierPoint bezierPointConstructor(float bezierPoints[][2], float t)
{
  // determines whcih bezier curve to find when working with splines
  int n = floor(t);
  float x1 = bezierPoints[0 + 4 * n][0];
  float x2 = bezierPoints[1 + 4 * n][0];
  float x3 = bezierPoints[2 + 4 * n][0];
  float x4 = bezierPoints[3 + 4 * n][0];

  float y1 = bezierPoints[0 + 4 * n][1];
  float y2 = bezierPoints[1 + 4 * n][1];
  float y3 = bezierPoints[2 + 4 * n][1];
  float y4 = bezierPoints[3 + 4 * n][1];
  // bernestein polynomials
  // subtracts the n value from t to get the t value for the bezier curve
  point derivativeBpoint = derivativeBezier(bezierPoints, t);
  t -= n;
  bezierPoint bezierPoint(
      /*x points*/ (x1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + x2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) + x3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + x4 * pow(t, 3)),
      /*y points*/ (y1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + y2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) + y3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + y4 * pow(t, 3)),
      /*t value*/ t + n,
      /*angle*/ RadToDeg(atan2(derivativeBpoint.y, derivativeBpoint.x)),
      /*curvature*/ curvature(bezierPoints, t + n));
  return (bezierPoint);
}

/**
 * @brief finds the distance of a point from a line
 * @param points an array containing the points of the line
 * @param point1 the index first point of the line
 * @param point2 the index second point of the line
 */
float perpendicularDist(std::vector<bezierPoint> points, int point1, int point2)
{
  point p1(points[point1].x, points[point1].y);
  point p2(points[point2].x, points[point2].y);

  point robot = point(X, Y);
  // calculate distance
  float A = p2.y - p1.y;
  float B = p1.x - p2.x;
  float C = p2.x * p1.y - p1.x * p2.y;
  float d = fabs((A * robot.x + B * robot.y + C)) / hypot(A, B);
  return d;
}

/**
 * @brief finds the sign of the distance bettwen two points
 * @param points an array containing the points
 * @param point1 the first point
 * @param point2 the second point
 */
int signOfDistance(std::vector<bezierPoint> points, int p1, int p2)
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
  float kp = .5;
  float ki = 0.000;
  float kd = 0.00;
  float ld;
  float minSpeed = 15;
  float maxSpeed = 90;
  float v;
  float kv = 5;
  float segmentDist;
  float pathHeading;
  float tSpeed;
  float ldAngle;
  float prevError = 0;
  float totalError = 0;
  int sign;
  int pointClosest = 0;
  float tval = 0;

  // Initializing the 1-D vector of struct "point"
  std::vector<bezierPoint> Bpoints(10, bezierPoint(0, 0, 0, 0, 0));
  // Filling the vector with points
  for (int i = 0; i < 10; i++)
  {
    Bpoints[i] = bezierPointConstructor(points, tval);
    tval += 0.005;
  }
  Brain.Screen.drawRectangle(0, 0, 480, 240, yellow);
  Brain.Screen.print("stanley in progress");
  while (true)
  {
    // find the closest point
    // pointClosest =closestPoint(points,8); //alternate way to find closest point
    pointClosest = closestPoint(Bpoints, 8);
    // if the closest point is the last point of the points array

    // std::cout<<"1"<<std::endl;
    // wh
    if (pointClosest != 0) // if you set this to one the entire thing breaks
    {
      // std::cout<<"1.5"<<std::endl;
      // if the closest point is not the first point, erase all prevoius values until it is
      Bpoints.erase(Bpoints.begin(), Bpoints.begin() + pointClosest);
      // fill the vecotr until the length is 20
      while (Bpoints.size() < 10)
      {
        Bpoints.push_back(bezierPointConstructor(points, tval));
        tval += 0.005;
      }
      pointClosest = 0;
    }

    if (Bpoints[pointClosest].tval > length / 4)
    {
      break;
    }
    // slope of the path at the closest point
    pathHeading = Bpoints[pointClosest].angle;
    //   std::cout<<"2"<<std::endl;
    // find the distance to the line segment after the closest point

    segmentDist = perpendicularDist(Bpoints, pointClosest, pointClosest + 1); // distance of the path segment after the closest point
    sign = signOfDistance(Bpoints, pointClosest, pointClosest + 1);
    //  std::cout<<"3"<<std::endl;
    // calculate lookahead distance

    v = ((RotationR.velocity(rpm) / 2) + (RotationL.velocity(rpm) / 2)) / 2; // take the average of the two sides converting rpm to percent
    v = fmax(v, minSpeed);                                               // if velocity is less than minSpeed, set it to minSpeed
    ld = v / kv;

    // calculate the angle to the global angle to the lookahead point
    ldAngle = RadToDeg(atan2(segmentDist, ld));

    // calculate the error
    float error = (ldAngle * sign + pathHeading - gyro1.yaw(deg));
    // std::cout<<"4"<<std::endl;
    // calculate the PID output

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

    float output = (error * kp) + (totalError)*ki + (prevError - error) * kd;
    //  std::cout<<"5"<<std::endl;
    prevError = error;
    // calculate the speed of the motors
    tSpeed = 90 - fabs((2 * error));
    // limit the value of tSpeed
    tSpeed = fmax(tSpeed, minSpeed);
    tSpeed = fmin(tSpeed, maxSpeed);
    // drive
    drive(tSpeed + output, tSpeed - output, 10);

    // print the values
    // pathDistance<<","<<ldAngle*-sign<<","<<perpendicularDist(points,0,1)<<","
    //  std::cout<<"x="<<X<<", y= "<<Y<<", pointclosest= "<<pointClosest<<" error= "<<error<<std::endl
    //  <<"seg1dist= "<<segment1dist<<", seg2dist= "<<segment2dist<<", pathdist= "<<pathDistance<<std::endl<<std::endl;
    std::cout << X << ",,," << Y << "," << tSpeed << "," << tval << std::endl;
    wait(15, msec);
  }
  drive_brake();
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
  std::cout << "done" << std::endl;
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
void curveDrive(double x, double y, double speed, double wt = 10)
{
  double dist = robotDistance(x, y);
  double errorX = x - X;
  double errorY = y - Y;
  double localX = errorX * cos(gyro1.yaw()) - (errorY * sin(gyro1.yaw()));
  ;
  double curvature = (2 * localX) / (dist * dist);
  float L = speed * (2 + (curvature * TrackWidth)) / 2;
  float R = speed * (2 - (curvature * TrackWidth)) / 2;
  drive(L, R, wt);
}
/**
 * @brief find the intersection of a line and a circle and returns the tvlaue of the intersection
 * @param points an array containing the points
 * @param lineSegment the index of the start of the line segment
 * @param ld the distance from the robot to the point
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
    float tval1 = (x_1 - x1) / (x2 - x1) + lineSegment;

    double x_2 = (-2 * m * b - sqrt(discriminant)) / (2 * (pow(m, 2) + 1));
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
  float ld = 6;
  int pointClosest = 0;
  float tval;
  float tvalI;
  float x1, y1;
  while (true)
  {
    pointClosest = closestPoint(points, length);
    if (length == pointClosest - 1)
    {
      break;
    }
    // find the lookahead point
    for (int i = 0; i < length; i++)
    {
      tvalI = lineCircleIntersection(points, i, ld);
      if (tvalI > tval)
      {
        pointLookahead = i;
        tval = tvalI;
      }
    }
    // std::cout<<tval<<","<<pointLookahead<<std::endl;
    tval = tval - pointLookahead;
    if (tval < 0)
    {
      // std::cout<<"tval is less than 0"<<std::endl;
      x1 = points[pointClosest][0];
      y1 = points[pointClosest][1];
    }
    else
    {
      // std::cout<<"tval is greater than 0"<<std::endl;
      x1 = points[pointLookahead][0] + tval * (points[pointLookahead + 1][0] - points[pointLookahead][0]);
      y1 = points[pointLookahead][1] + tval * (points[pointLookahead + 1][1] - points[pointLookahead][1]);
    }
    std::cout << x1 << "," << y1 << "," << X << "," << Y << std::endl;
    curveDrive(x1, y1, 25);
  }
}
