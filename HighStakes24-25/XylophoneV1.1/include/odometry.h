#pragma once

class point
{
public:
  float x;
  float y;
  point(float x, float y) : x(x), y(y) {}
  point() : x(0), y(0) {}
  float distance(point p);
  point operator+(point p1);
  point operator+=(point p1);
  float robotDistance();
  bool equals(point p1);
  float angleTo(point p1);
  void print();
};

#include "monteCarlo.h"

pose odomStep(float dx, float dy,float heading, float deltaHeading);
extern point robot;
int odometery();