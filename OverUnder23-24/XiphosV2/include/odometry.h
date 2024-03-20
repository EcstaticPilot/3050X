#pragma once

class point
{
public:
  float x;
  float y;
  point(float x, float y) : x(x), y(y) {}
  point() : x(0), y(0) {}
  float distance(point p);
  float robotDistance();
  void print();
};
extern point robot;
int odometery();