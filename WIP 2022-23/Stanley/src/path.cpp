#include "vex.h"
#include <vector>
#include <iomanip>
const float TrackWidth = 8.5;
// start off path stuff
using namespace vex;
// an explanation for error works
/*
The robot knows where it is at all times. It knows this because it knows where it was. By adding where it was from where it isn't, or where it isn't from where it is
(whichever is greater), it obtains a difference, or deviation. The guidance subsystem uses deviations to generate corrective commands to drive the robot from a position where
it is to a position where it isn't, and arriving at a position where it wasn't, it now is. Consequently, the position where it is, is now the position that it wasn't, and
it follows that the position that it was, is now the position that it isn't. In the event that the position that it is in is not the position that it wasn't,
the system has acquired a variation, the variation being the difference between where the robot is, and where it wasn't. If variation is considered to be a significant factor,
it too may be corrected by the GEA. However, the robot must also know where it was. The robot guidance computer scenario works as follows.
Because a variation has modified some of the information the robot has obtained, it is not sure just where it is. However, it is sure where it isn't, within reason,
and it knows where it was. It now subtracts where it should be from where it wasn't, or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be,
and where it was, it is able to obtain the deviation and its variation, which is called error."

*/

class point
{
public:
  float x;
  float y;
  point(float x, float y) : x(x), y(y) {}

  float robotDistance(float x, float y)
  {
    return (hypot(X - x, Y - y));
  }
};

/**
 * @brief convert radians to degrees
 * @param rad radians(float)
 */
float RadToDeg(float rad)
{
  return (rad * 180 / M_PI);
}

int signOf(float x)
{
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}
/**
 * @brief convert degrees to radians
 * @param deg degrees (float)
 * @return radians (float)
 */
float DegToRad(float deg)
{
  return (deg * M_PI / 180);
}

/** @brief checks if 2 value have different signs
 * @param a first value
 * @param b second value
 * @return true if they are different, false if they are the same sign
 */
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

/**
 * @brief finds the distance between the robot and a point
 * @param x x coordinate of the point
 * @param y y coordinate of the point
 */
float robotDistance(float x, float y)
{
  return (hypot(X - x, Y - y));
}

/*
███████╗  ████████╗   █████╗   ███╗   ██╗  ██╗       ███████╗  ██╗   ██╗
██╔════╝  ╚══██╔══╝  ██╔══██╗  ████╗  ██║  ██║       ██╔════╝  ╚██╗ ██╔╝
███████╗     ██║     ███████║  ██╔██╗ ██║  ██║       █████╗     ╚████╔╝
╚════██║     ██║     ██╔══██║  ██║╚██╗██║  ██║       ██╔══╝      ╚██╔╝
███████║     ██║     ██║  ██║  ██║ ╚████║  ███████╗  ███████╗     ██║
╚══════╝     ╚═╝     ╚═╝  ╚═╝  ╚═╝  ╚═══╝  ╚══════╝  ╚══════╝     ╚═╝
*/

/** @brief finds the derivative of a bezier curve
 * @param bezierPoints an array containing the control points
 * @param t the tvalue
 * @return a point struct with the x and y of the derivatuve point
 */
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

/// @brief finds the curvature of a bezier curve
/// @param bezierPoints an array containing the control points
/// @param t the tval of the point
/// @return a float containing the curvature
float findcurvature(float bezierPoints[][2], float t)
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

class bezierPoint
{
public:
  float x;
  float y;
  float tval;
  float angle;
  float curvature;
  bezierPoint(float x, float y, float tval, float angle, float curvature) : x(x), y(y), tval(tval), angle(angle), curvature(curvature) {}
  bezierPoint(float bezierPoints[][2], float t)
  {
    int n = floor(t);
    float x1 = bezierPoints[0 + 4 * n][0]; // a
    float x2 = bezierPoints[1 + 4 * n][0]; // b
    float x3 = bezierPoints[2 + 4 * n][0]; // c
    float x4 = bezierPoints[3 + 4 * n][0]; // d

    float y1 = bezierPoints[0 + 4 * n][1]; // e
    float y2 = bezierPoints[1 + 4 * n][1]; // f
    float y3 = bezierPoints[2 + 4 * n][1]; // g
    float y4 = bezierPoints[3 + 4 * n][1]; // h
    // X=i
    // Y=j
    //  bernestein polynomials
    //  subtracts the n value from t to get the t value for the bezier curve
    point derivativeBpoint = derivativeBezier(bezierPoints, t);
    t -= n;
    bezierPoint bezierPoint(
        this->x = (x1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + x2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) + x3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + x4 * pow(t, 3)),
        this->y = (y1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + y2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) + y3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + y4 * pow(t, 3)),
        this->tval = t + n,
        this->angle = RadToDeg(atan2(derivativeBpoint.x, derivativeBpoint.y)),
        this->curvature = findcurvature(bezierPoints, t + n));
  }
};
/*
0 = (m*((a(-t^3+3t^2-3t+1)+b(3t^3-6t^2+3t)+c(-3t^3+3t^2)+d(t^3))*m - e(-t^3+3t^2-3t+1)+f(3t^3-6t^2+3t)+g(-3t^3+3t^2)+h(t^3)+j)+i)/(m^2+1)    -    a(-t^3+3t^2-3t+1)+b(3t^3-6t^2+3t)+c(-3t^3+3t^2)+d(t^3)


0=m^3⋅(a(−t^3+3t^2−3t+1)+b(3t^3−6t^2+3t)+c(−3t^3+3t^2)+d(t^3))−m^2⋅(a(−t^3+3t^2−3t+1)+b(3t^3−6t^2+3t)+c(−3t^3+3t^2)+d(t^3))−(a(−t^3+3t^2−3t+1)+b(3t^3−6t^2+3t)+c(−3t^3+3t^2)+d(t^3))+(e(−t^3+3t^2−3t+1)−f(3t^3−6t^2+3t)−g(−3t^3+3t^2)−h(t^3))+j+i


*/

/**
 * @brief finds the closest point to the robot
 * @param points an array containing the points as a struct
 * @return the index of the closest point
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
 * @brief finds the distance of a point from a line
 * @param points an array containing the points of the line
 * @param point1 the index first point of the line
 * @param point2 the index second point of the line
 * @return the distance from the point to the line
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
 * @return an integer representing the sign of the distance
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
 * @param isReversed if the robot is reversed when following the path
 *
 */
void stanley(float points[][2], int length, bool isReversed)
{
  bool isPID = true;
  float kf = 0;
  float robotlength = 10;
  float width = 9;

  float waitTime = 10;
  // constants for reaction to error
  // untested values
  float kp = 2;

  float ki = 0;

  float kd = 6.25; // 6.666666

  float kv = 10; // v/kv   7.5
  // it would apeear that kd=kp*5 is good for some reason
  //  lookahead distance
  float ld;
  int lead = 0;
  // minimum and maximum speed
  float minSpeed = 25;
  float maxSpeed = 100;
  float v;

  // konstant for how much error changes the speed
  float ke = 0;
  // konstant for how much the curvature changes the speed
  float kc = 0;

  float radius;
  float totalRadius;

  float segmentDist;
  float pathHeading;
  float robotAngle;
  float tSpeed;
  float ldAngle;

  float prevError = 0;
  float totalError = 0;
  float error = 0;

  float vL;
  float vR;
  float normFactor;
  float output;

  int sign;
  int bezierCount = floor(length / 4);
  int pointClosest = 0;
  float tval = 0;

  // the 1-D vector ofP struct "point"
  std::vector<bezierPoint> Bpoints(10, bezierPoint(0, 0, 0, 0, 0));
  // Filling the vector with points
  for (int i = 0; i < 10; i++)
  {
    Bpoints[i] = bezierPoint(points, tval);
    tval += 0.0025;
  }
  Brain.Screen.drawRectangle(0, 0, 480, 240, yellow);
  Brain.Screen.print("stanley in progress");

  Brain.Timer.reset();
  std::cout << "kp=" << kp << ",ki=" << ki << ",kd=" << kd << ",kv=" << kv << ",ke=" << ke << ",kc=" << kc << std::endl;
  std::cout << "X,Y,error" << std::endl;
  while (true)
  {
    // find the closest point to the robot
    pointClosest = closestPoint(Bpoints, 9);

    // if the closest point is the last point of the points array
    if (pointClosest != 0) // if you set this to one the entire thing breaks  ¯⁠\⁠_⁠(⁠ツ⁠)⁠_⁠/⁠¯
    {
      // if the closest point is not the first point, erase all prevoius values until it is
      Bpoints.erase(Bpoints.begin(), Bpoints.begin() + pointClosest);

      // fill the vecotr until the length is 10
      while (Bpoints.size() < 10)
      {
        Bpoints.push_back(bezierPoint(points, tval));
        tval += 0.0025;
      }
      // set the pointClosest to 0
      pointClosest = 0;
    }
    // check if the tval is past the lenth of the total bezier spline. the 0.001 exists because computers are bad at math and error builds up due to thats
    if (Bpoints[1].tval >= bezierCount)
    {
      break;
    }

    // slope of the path at the closest point
    pathHeading = Bpoints[0 + lead].angle;

    // find the distance to the line segment after the closest point
    segmentDist = perpendicularDist(Bpoints, 0 + lead, 1 + lead); // distance of the path segment after the closest point

    // find the sign of the distance
    sign = signOfDistance(Bpoints, 0 + lead, 1 + lead);

    // calculate lookahead distance
    v = ((RotationR.velocity(rpm) / 2) + (RotationL.velocity(rpm) / 2)) / 2; // take the average of the two sides converting rpm to percent
    v = fmax(fabs(v), minSpeed);                                             // if velocity is less than minSpeed, set it to minSpeeds

    // calculate the lookahead distance
    ld = (v / kv);
    // ld=7.5;
    //   calculate the angle to the global angle to the lookahead point
    ldAngle = RadToDeg(atan2(segmentDist, ld));

    // find the robot angle and limit it to 360 idk if it matters or not. it probably does since rotation is uncapped
    robotAngle = fmod(gyro1.rotation(degrees), 360);

    if (isReversed)
    {
      robotAngle = fmod(gyro1.rotation(degrees) + 180, 360);
    }

    // robotAngle = gyro1.rotation(degrees);

    prevError = error;
    // calculate the error
    error = (ldAngle * sign + pathHeading - robotAngle);
    // if the error is greater than 180, subtract 360 from it and make sure its pointing the right way
    error = fmod(error, 360);
    if (fabs(error) > 180)
    {
      error = error - (signOf(error) * 360);
    }

    // reset integral if error crosses zero
    if (zeroCrossing(error, prevError))
    {
      totalError = 0;
    }
    else
    {
      totalError += error;
    }

    // calculate the PID output

    // set the previous error to the current error

    // calculate the speed of the motors
    tSpeed = 100 - (fabs(Bpoints[1].curvature * kc) + fabs(error * ke));

    // limit the value of tSpeed using fmax and fmin

    tSpeed = fmax(tSpeed, minSpeed);
    tSpeed = fmin(tSpeed, maxSpeed);
    // tval is between 0.1 and 0, start slowing down
    if (Bpoints[1].tval > (bezierCount - 0.05))
    {
      tSpeed = (bezierCount - Bpoints[0].tval) * 2000;
    }

    // calculate the speed of the left and right motors
    if (isPID)
    {
      output = (error)*kp + (totalError)*ki + (error - prevError) * kd;

      if (kf > 0)
      {

        float pathRadius = 1 / (Bpoints[0].curvature * kf);
        float rl = pathRadius + (width / 2);
        float rr = pathRadius - (width / 2);
        float ratio = rl / rr;
        vL = (tSpeed * ratio) + output;
        vR = tSpeed - output;
      }
      else
      {
        vL = tSpeed + output;
        vR = tSpeed - output;
      }
    }
    else
    {
      radius = (robotlength / 2) * tanf(DegToRad(90 - error) + 0.0001);
      if (kf > 0)
      {
        float curvature = 1 / radius;
        radius = 1 / (curvature + (Bpoints[0].curvature * kf));
      }
      float rl = radius + (width / 2);
      float rr = radius - (width / 2);
      float ratio = rl / rr;
      vL = tSpeed * ratio;
      vR = tSpeed;
    }
    // calculate the normalization factor
    normFactor = maxSpeed / fmax(fabs(vL), fabs(vR));

    // clip speed if necessary
    if (normFactor < 1)
    {
      vL *= normFactor;
      vR *= normFactor;
    }

    // drive using voltDrive
    if (!isReversed)
    {
      voltDrive(vL, vR, 0);
    }
    else
    {
      voltDrive(-vR, -vL, 0);
    }

    // std::cout << error << "," << vL <<","<<vR<< "\n";
    std::cout << X << "," << Y << "," << error << std::endl;
    wait(waitTime, msec);
    // std::cout <<Brain.Timer.time()<<","<< error << "," << output << "," << kp * error << "," << ki * totalError << "," << kd * (error - prevError) << std::endl; // pid tuning config
    // wait(15, msec);
  }
  drive_brake(brake);
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
  std::cout << "done" << std::endl;
}

/*
██████╗   ██╗   ██╗  ██████╗   ███████╗      ██████╗   ██╗   ██╗  ██████╗   ███████╗  ██╗   ██╗  ██╗  ████████╗
██╔══██╗  ██║   ██║  ██╔══██╗  ██╔════╝      ██╔══██╗  ██║   ██║  ██╔══██╗  ██╔════╝  ██║   ██║  ██║  ╚══██╔══╝
██████╔╝  ██║   ██║  ██████╔╝  █████╗        ██████╔╝  ██║   ██║  ██████╔╝  ███████╗  ██║   ██║  ██║     ██║
██╔═══╝   ██║   ██║  ██╔══██╗  ██╔══╝        ██╔═══╝   ██║   ██║  ██╔══██╗  ╚════██║  ██║   ██║  ██║     ██║
██║       ╚██████╔╝  ██║  ██║  ███████╗      ██║       ╚██████╔╝  ██║  ██║  ███████║  ╚██████╔╝  ██║     ██║
╚═╝        ╚═════╝   ╚═╝  ╚═╝  ╚══════╝      ╚═╝        ╚═════╝   ╚═╝  ╚═╝  ╚══════╝   ╚═════╝   ╚═╝     ╚═╝
*/
/**
 * @brief calculates if the distance between the next point is closer than the intial point
 * @param initial the initial point
 * @param points an array containing the points
 * @return the next point
 */
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
  voltDrive(L, R, wt);
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
