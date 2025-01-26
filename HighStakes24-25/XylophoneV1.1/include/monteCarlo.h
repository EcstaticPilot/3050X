
#pragma once
#include "vex.h"
#include <vector>
#include <random>
#include "odometry.h"

class pose : public point
{
public:
    float theta;
    pose(float x, float y, float theta) : point(x, y), theta(theta) {}
};
class sample : public pose
{
public:
    float w;
    sample(float x, float y, float theta, float w) : pose(x, y, theta), w(w) {}
    sample(pose p1, float w) : pose(p1), w(w) {}
    sample(): pose(0,0,0), w(0) {}
};
class MonteCarlo
{
public:
    //samples
    static std::vector<sample> samples;
    static std::mt19937 randgen;
    // max samples
    static int J;

    static void init(pose start, int max_samples);
    static void resample();
    static void update();
    static void getPose();
};