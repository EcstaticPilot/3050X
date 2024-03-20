#pragma once
#include "vex.h"
#include <vector>
point derivativeBezier(std::vector<point> controlPoints, float t);
float RadToDeg(float rad);
float findcurvature(std::vector<point> controlPoints, float t);
class bezierPoint : public point
{
public:
  float tval;
  float angle;
  float curvature;
  bezierPoint(float x, float y, float tval, float angle, float curvature) : point(x, y), tval(tval), angle(angle), curvature(curvature) {}

  bezierPoint(std::vector<point> controlPoints, float t)
  {
    // find which segment of the spline the t value is in
    int n = floor(t);

    float x1 = controlPoints[0 + 4 * n].x;
    float x2 = controlPoints[1 + 4 * n].x;
    float x3 = controlPoints[2 + 4 * n].x;
    float x4 = controlPoints[3 + 4 * n].x;

    float y1 = controlPoints[0 + 4 * n].y;
    float y2 = controlPoints[1 + 4 * n].y;
    float y3 = controlPoints[2 + 4 * n].y;
    float y4 = controlPoints[3 + 4 * n].y;

    // get the derivative point of the bezier curve
    point derivativeBpoint = derivativeBezier(controlPoints, t);

    //  subtracts the n value from t to get the t value for the bezier curve
    t -= n;

    bezierPoint bezierPoint(
        this->x = (x1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + x2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) +
                   x3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + x4 * pow(t, 3)),
        this->y = (y1 * (pow(-t, 3) + 3 * pow(t, 2) - 3 * t + 1) + y2 * (3 * pow(t, 3) - 6 * pow(t, 2) + 3 * t) +
                   y3 * (-3 * pow(t, 3) + 3 * pow(t, 2)) + y4 * pow(t, 3)),
        this->tval = t + n,
        this->angle = RadToDeg(atan2(derivativeBpoint.x, derivativeBpoint.y)),
        this->curvature = findcurvature(controlPoints, t + n));
  }
};

class Stanley
{
private:
  static std::vector<point> controlPoints;
  static std::vector<bezierPoint> bezierPoints;
  static float tval;
  static int length;
  static int bezierCount;
  static int closestPoint();
  static float perpendicularDist(int p1, int p2);
  static int signOfDistance(int p1, int p2);

public:
  static void setPath(std::vector<point> controlPoints);
  static void run(stanleyConfig config = fast, bool isReversed = false);
  static void toPoint(point p, float finalAngle, stanleyConfig config = fast, float startDist =12, float endDist =12, bool reverse = false);
};