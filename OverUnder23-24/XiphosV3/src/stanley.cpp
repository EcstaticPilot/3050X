#include "vex.h"
#include "pid.h"
#include <vector>
#include <iomanip>
#include <algorithm>
// start off path stuff
using namespace vex;
#include "stanley.h"
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
// declare the static variables
std::vector<stanleyEvent> Stanley::stanleyEvents;
std::vector<point> Stanley::controlPoints;
std::vector<bezierPoint> Stanley::bezierPoints;
float Stanley::tval = 0;
int Stanley::length = 0;
int Stanley::bezierCount = 0;
float Stanley::timeout = 10000;
float Stanley::slowdownStart = 0.1;
float point::distance(point p)
{
  return (hypot(this->x - p.x, this->y - p.y));
}
float point::robotDistance()
{
  return distance(robot);
}

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
float average(std::vector<float> vector)
{
  float sum = 0;
  for (int i = 0; i < vector.size(); i++)
  {
    sum += vector[i];
  }
  return sum / (vector.size());
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

  int A = signOf(a);
  int B = signOf(b);
  if (A != B)
  {
    return true;
  }
  return false;
}

float robotDistance(float x, float y)
{
  return hypot(robot.x - x, robot.y - y);
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
 * @return a point struct with the x and y of the derivative point
 */
point derivativeBezier(std::vector<point> controlPoints, float t)
{
  int n = floor(t);
  float x1 = controlPoints[0 + 4 * n].x;
  float x2 = controlPoints[1 + 4 * n].x;
  float x3 = controlPoints[2 + 4 * n].x;
  float x4 = controlPoints[3 + 4 * n].x;

  float y1 = controlPoints[0 + 4 * n].y;
  float y2 = controlPoints[1 + 4 * n].y;
  float y3 = controlPoints[2 + 4 * n].y;
  float y4 = controlPoints[3 + 4 * n].y;

  // subtracts the n value from t to get the t value for the bezier curve
  t -= n;
  point bezierPoint(
      // x points
      (x1 * (-3 * pow(t, 2) + 6 * t - 3) + x2 * (9 * pow(t, 2) - 12 * t + 3) +
       x3 * (-9 * pow(t, 2) + 6 * t) + x4 * 3 * pow(t, 2)),
      // y points
      (y1 * (-3 * pow(t, 2) + 6 * t - 3) + y2 * (9 * pow(t, 2) - 12 * t + 3) +
       y3 * (-9 * pow(t, 2) + 6 * t) + y4 * 3 * pow(t, 2)));
  return (bezierPoint);
}

/** @brief finds the curvature of a bezier curve
 * @param BezierPoints an array containing the control points
 * @param t the tval of the point
 * @return a float containing the curvature
 */
float findcurvature(std::vector<point> controlPoints, float t)
{
  int n = floor(t);
  float x0 = controlPoints[0 + 4 * n].x;
  float x1 = controlPoints[1 + 4 * n].x;
  float x2 = controlPoints[2 + 4 * n].x;
  float x3 = controlPoints[3 + 4 * n].x;

  float y0 = controlPoints[0 + 4 * n].y;
  float y1 = controlPoints[1 + 4 * n].y;
  float y2 = controlPoints[2 + 4 * n].y;
  float y3 = controlPoints[3 + 4 * n].y;
  // bernestein polynomials
  // subtracts the n value from t to get the t value for the bezier curve
  t -= n;
  float dt = 1 - t;

  // Calculate the components of the first and second derivatives
  float dx_dt = (3 * pow(dt, 2) * (x1 - x0) + 6 * dt * t * (x2 - x1)) + 3 * pow(t, 2) * (x3 - x2);
  float dy_dt = (3 * pow(dt, 2) * (y1 - y0) + 6 * dt * t * (y2 - y1)) + 3 * pow(t, 2) * (y3 - y2);
  float d2x_dt2 = (6 * dt * (x2 - 2 * x1 + x0)) + (6 * t * (x3 - 2 * x2 + x1));
  float d2y_dt2 = (6 * dt * (y2 - 2 * y1 + y0)) + (6 * t * (y3 - 2 * y2 + y1));

  // Calculate the curvature
  float numerator = fabs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2);
  float denominator = pow(dx_dt * dx_dt + dy_dt * dy_dt, 1.5);
  float curvature = numerator / denominator;
  return curvature;
}

/**
 * @brief finds the closest point to the robot
 * @param points an array containing the points as a struct
 * @return the index of the closest point
 */
int Stanley::closestPoint()
{
  float min = std::numeric_limits<float>::max();
  ;
  int index = 0;
  float dist;
  // std::cout<<"start"<<std::endl;

  for (int i = 0; i < bezierPoints.size(); i++)
  {
    dist = bezierPoints[i].robotDistance();
    // std::cout<<"dist "<<i<<" = "<<dist<<std::endl;
    // std::cout<<"min="<<min<<(dist<min)<<std::endl;
    if (dist < min)
    {
      min = dist;
      index = i;
      // std::cout<<"newmin "<<min<<" index "<<index<<std::endl;
    }
  }
  return index;
}
/**
 * @brief add event
 *
 * @param funcptr
 * @param tval
 */
void Stanley::addEvent(void (*funcptr)(), float tval)
{
  stanleyEvents.push_back(stanleyEvent(funcptr, tval));
}

void Stanley::setTimeout(float time)
{
  Stanley::timeout = time;
}
/**
 * @brief finds the distance of a point from a line
 * @param points an array containing the points of the line
 * @param point1 the index first point of the line
 * @param point2 the index second point of the line
 * @return the distance from the point to the line
 */
float Stanley::perpendicularDist(int point1, int point2)
{
  point p1 = point(bezierPoints[point1]);
  point p2 = point(bezierPoints[point2]);

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
int Stanley::signOfDistance(int p1, int p2)
{
  float x1 = bezierPoints[p1].x;
  float y1 = bezierPoints[p1].y;
  float x2 = bezierPoints[p2].x;
  float y2 = bezierPoints[p2].y;
  float x3 = robot.x;
  float y3 = robot.y;
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
void Stanley::setPath(std::vector<point> controlPointsi)
{
  //reset everything
  Stanley::controlPoints.reserve(controlPointsi.size());
  Stanley::controlPoints = controlPointsi;
  Stanley::length = Stanley::controlPoints.size();
  Stanley::bezierCount = floor(length / 4);
  Stanley::tval = 0;
  bezierPoints.clear();
  bezierPoints.reserve(10);
  // Filling the vector with points
  for (int i = 0; i < 10; i++)
  {
    bezierPoints.push_back(bezierPoint(Stanley::controlPoints, Stanley::tval));
    Stanley::tval += 0.0025;
  }
  float distTarget = 10;
  float currDist = 0;
  point prevPoint = controlPointsi[bezierCount*4 - 1];
  //calcl slowdown start
  for(int i = bezierCount; i>0; i-=0.025){
    point currPoint = bezierPoint(controlPointsi, i);
    currDist += currPoint.distance(prevPoint);
    prevPoint = currPoint;
    if(currDist>distTarget){
      Stanley::slowdownStart = Stanley::bezierCount - i;
      break;
    }
  }
  return;
}

/**
 * @brief stanley controller for following a path of points
 * @param points an array containing points to follow
 * @param length the length of the array
 * @param isReversed if the robot is reversed when following the path
 *
 */
void Stanley::run(stanleyConfig config, bool isReversed)
{
  std::cout << "start" << std::endl;
  bool isPID = true;
  float kf = 0;
  float robotlength = 10.5;
  float width = 10.5;
  vex::timer Timer = vex::timer();
  point prevRobot = robot;
  float waitTime = 10;
  // constants for reaction to error PID
  float kp, ki, kd, kv, ke, kc, minSpeed, maxSpeed;
  std::cout << "size" << bezierPoints.size() << std::endl;
  // how far from the end of the path to start slowing down

  switch (config)
  {
  case (fast):
  {
    kp = 2.5;
    ki = 0;
    kd = 8; // 10
    kv = 10;
    minSpeed = 50;
    maxSpeed = 100;
    ke = 0;
    kc = 0;
    break;
  }
  case (meduim):
  {
    kp = 1.5;
    ki = 0;
    kd = 4;
    kv = 10;
    minSpeed = 25;
    maxSpeed = 75;
    ke = 0;
    kc = 500;
    break;
  }
  case (slow):
  {
    kp = 1.5;
    ki = 0;
    kd = 8;
    kv = 10;
    minSpeed = 25;
    maxSpeed = 50;
    ke = 0;
    kc = 0;
    break;
  }
  }

  float v;
  float radius;
  float ld;
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
  int pointClosest = 0;

  std::vector<float> history(10, 0);

  Brain.Screen.print("stanley in progress");

  std::cout << "kp=" << kp << ",ki=" << ki << ",kd=" << kd << ",kv=" << kv << ",ke=" << ke << ",kc=" << kc << std::endl;

  while (true)
  {
    // find the closest point to the robot
    pointClosest = closestPoint();

    //  if the closest point is the not the first point of the points array
    if (pointClosest != 0)
    {
      // since closest point is not the first point, erase all prevoius values until it is
      bezierPoints.erase(bezierPoints.begin(), bezierPoints.begin() + pointClosest);

      //  fill the vector until the length is 10
      while (bezierPoints.size() < 12)
      {
        bezierPoints.push_back(bezierPoint(controlPoints, tval));
        tval += 0.0025;
      }
      // set the pointClosest to 0
      pointClosest = 0;
    }

    // check if the tval is past the lenth of the total bezier spline. if it is, break the loop
    if (bezierPoints[1].tval >= bezierCount)
    {
      std::cout << "BREAK_STANLEY";
      break;
    }
    //std::cout << robot.x << "," << robot.y << std::endl;
    // slope of the path at the closest point
    pathHeading = bezierPoints[0].angle;

    // find the distance to the line segment after the closest point
    segmentDist = perpendicularDist(0, 1);

    // find the sign of the distance
    sign = signOfDistance(0, 1);

    //calculate the velocity of the robot in percent using the average speeds of the motors
    v = (RB.velocity(pct) + LB.velocity(pct)) / 2;

    //if velocity is less than minSpeed, set it to the minSpeed
    v = fmax(fabs(v), minSpeed);
    v = maxSpeed;
    // calculate the lookahead distance
    ld = (v / kv);

    // calculate angle to the lookahead point relative to path angle
    ldAngle = RadToDeg(atan2(segmentDist, ld));

    // find the robot angle and limit it to 360 since rotation is uncapped
    robotAngle = fmod(gyro1.rotation(degrees), 360);

    // if the robot is following the path backward, flip around the angle
    if (isReversed)
    {
      robotAngle = fmod(gyro1.rotation(degrees) + 180, 360);
    }

    // calculate the error
    error = (ldAngle * sign + pathHeading - robotAngle);

    //  mod error by 360 to make sure the robot isnt turning a full circle
    error = fmod(error, 360);

    // if the error is greater than 180, subtract 360 from it and make sure its pointing the right way
    if (fabs(error) > 180)
    {
      error = error - (signOf(error) * 360);
    }

    // calculate the speed of the motors
    tSpeed = maxSpeed - (fabs(bezierPoints[1].curvature * kc) + fabs(error * ke));

    // limit the value of tSpeed using fmax and fmin
    tSpeed = fmax(tSpeed, minSpeed);

    // if the robot is toward the end of the path, start slowing down
    if (bezierPoints[1].tval > (bezierCount - slowdownStart))
    {
      tSpeed = (bezierCount - bezierPoints[0].tval) * (1 / slowdownStart) * maxSpeed + 10;
    }
    tSpeed = fmin(tSpeed, maxSpeed);

    // if PID control is enabled
    if (isPID)
    {
      // PID equation
      output = (error)*kp + (totalError)*ki + (error - average(history)) * kd;

      // calculate the left and right motor values
      vL = tSpeed + output;
      vR = tSpeed - output;
      history.push_back(error);
      while (history.size() > 9)
      {
        history.erase(history.begin());
      }
    }
    else
    {
      // geometric equation
      error = fmax(error, -89);
      error = fmin(error, 89);
      radius = (robotlength / 2) * tanf(DegToRad(90 - error) + 0.0001);

      // if feedforward is enbaled, use the path curvature
      if (kf > 0)
      {
        float curvature = 1 / radius;
        radius = 1 / (curvature + (bezierPoints[0].curvature * kf));
      }

      // calculate the turning radius of the left and right motors
      float rl = radius + (width / 2);
      float rr = radius - (width / 2);

      // calculate the ratio between the 2 radii
      float ratio = rl / rr;

      // calculate the left and right motor values
      vL = tSpeed * ratio;
      vR = tSpeed;
      // std::cout<<"error"<<error<<"radius"<<radius<<"ratio"<<ratio<<"vl "<<vL<<"vR" <<vR<<std::endl;
    }

    // reset integral if error crosses zero
    if (zeroCrossing(error, prevError))
    {
      totalError = 0;
    }
    else // otherwise add the error to the total error
    {
      totalError += error;
    }

    // set the previous error to the error
    prevError = error;

    // calculate the normalization factor
    normFactor = maxSpeed / fmax(fabs(vL), fabs(vR));

    // clip speed if necessary
    if (normFactor < 1)
    {
      vL *= normFactor;
      vR *= normFactor;
    }

    // drive using voltDrive
    if (isReversed == false)
    {
      voltDrive(vL, vR, 0);
    }
    else // if the robot is reversed, flip the left and right motor values
    {
      voltDrive(-vR, -vL, 0);
    }

    // check for events
    if (Stanley::stanleyEvents.empty() == false)
    {
      for (int i = 0; i < stanleyEvents.size(); i++)
      {
        // std::cout<<"tval"<<stanleyEvents[i].tval<<","<<tval<<std::endl;
        if (tval >= stanleyEvents[i].tval)
        {
          // std::cout<<"run";
          stanleyEvents[i].run();
          stanleyEvents.erase(stanleyEvents.begin() + i);
        }
      }
    }
    std::cout << robot.equals(prevRobot) << "," << (Timer.value() > Stanley::timeout) << std::endl;
    if (robot.equals(prevRobot) && Timer.value() > Stanley::timeout)
    {
      std::cout << "timeout" << std::endl;
      break;
    }
    prevRobot = robot;
    // std::cout << robot.x << ",,," << robot.y << std::endl;
    // std::cout << robot.x << ",,," << robot.y << "," << error << "," << output << "," << (error)*kp << "," << (totalError)*ki << "," << (error - prevError) * kd << std::endl;

    this_thread::sleep_for(waitTime);

    // std::cout <<Brain.Timer.time()<<","<< error << "," << output << "," << kp * error << "," << ki * totalError << "," << kd * (error - prevError) << std::endl; // pid tuning config
  }

  drive_brake(brake);
  Brain.Screen.drawRectangle(0, 0, 480, 240, green);
  std::cout << "done" << std::endl;
}

/**
 * @brief stanley drive to a point
 *
 * @param p point to drive to
 * @param finalAngle final angle of the path
 * @param config
 * @param startDist
 * @param endDist
 */
void Stanley::toPoint(point p, float finalAngle, stanleyConfig config, float startDist, float endDist, bool reverse)
{
  float angle;
  if (reverse)
  {
    angle = fmod(gyro1.rotation(degrees) + 180, 360);
  }
  else
  {
    angle = gyro1.rotation(degrees);
  }
  point p1 = point(robot);
  point p2 = point(robot.x + startDist * cos(angle), robot.y + startDist * sin(angle));
  point p3 = point(p.x + endDist * cos(finalAngle), p.y + endDist * sin(finalAngle));
  point p4 = point(p);
  std::vector<point> path = std::vector<point>{p1, p2, p3, p4};
  Stanley::setPath(path);
  Stanley::run(config, reverse);
}

/*
██████╗   ██╗   ██╗  ██████╗   ███████╗      ██████╗   ██╗   ██╗  ██████╗   ███████╗  ██╗   ██╗  ██╗  ████████╗
██╔══██╗  ██║   ██║  ██╔══██╗  ██╔════╝      ██╔══██╗  ██║   ██║  ██╔══██╗  ██╔════╝  ██║   ██║  ██║  ╚══██╔══╝
██████╔╝  ██║   ██║  ██████╔╝  █████╗        ██████╔╝  ██║   ██║  ██████╔╝  ███████╗  ██║   ██║  ██║     ██║
██╔═══╝   ██║   ██║  ██╔══██╗  ██╔══╝        ██╔═══╝   ██║   ██║  ██╔══██╗  ╚════██║  ██║   ██║  ██║     ██║
██║       ╚██████╔╝  ██║  ██║  ███████╗      ██║       ╚██████╔╝  ██║  ██║  ███████║  ╚██████╔╝  ██║     ██║
╚═╝        ╚═════╝   ╚═╝  ╚═╝  ╚══════╝      ╚═╝        ╚═════╝   ╚═╝  ╚═╝  ╚══════╝   ╚═════╝   ╚═╝     ╚═╝
*/
//! doesnt work

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
  double errorX = x - robot.x;
  double errorY = y - robot.y;
  double localX = errorX * cos(gyro1.yaw()) - (errorY * sin(gyro1.yaw()));
  ;
  double curvature = (2 * localX) / (dist * dist);
  float L = speed * (2 + (curvature * TrackWidth)) / 2;
  float R = speed * (2 - (curvature * TrackWidth)) / 2;
  voltDrive(L, R, wt);
}

void curveDrivePID(double x, double y, float accuracy)
{
  double dist = robotDistance(x, y);
  double errorX = x - robot.x;
  double errorY = y - robot.y;
  double localX = errorX * cos(gyro1.yaw()) - (errorY * sin(gyro1.yaw()));
  ;
  double curvature = (2 * localX) / (dist * dist);
  PID pid = PID(5.75, 0, 7, 0);
  float avgPos;
  while (true)
  {
    avgPos = ((LB.position(turns) + RB.position(turns)) / 2) * 3 / 5; // average position of the left and right encoders w/ gear ratio
    pid.error = dist - (avgPos * M_TWOPI * 1.625);
    if (fabs(pid.error) < accuracy)
    {
      break;
    }
    pid.update();
    int speed = pid.output;
    float L = speed * (2 + (curvature * TrackWidth)) / 2;
    float R = speed * (2 - (curvature * TrackWidth)) / 2;
    voltDrive(L, R, 10);
  }
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

  float h = robot.x;
  float k = robot.y;
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
    std::cout << x1 << "," << y1 << "," << robot.x << "," << robot.y << std::endl;
    curveDrive(x1, y1, 25);
  }
}
