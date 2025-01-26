
#pragma once
#include <vector>
#include <random>
#include "odometry.h"
#include <SDL2/SDL.h>
extern const double SCALE_FACTOR;
class pose : public point
{
public:
    float theta;
    pose(float x, float y, float theta) : point(x, y), theta(theta) {}
    pose(point pt, float theta) : point(pt), theta(theta) {}
    pose() : point(0, 0), theta(0) {}
    void print();
};
class sample : public pose
{
public:
    float w;
    sample(float x, float y, float theta, float w) : pose(x, y, theta), w(w) {}
    sample(pose p1, float w) : pose(p1), w(w) {}
    sample() : pose(0, 0, 0), w(0) {}
};
class MonteCarlo
{
public:
    // samples
    static std::vector<sample> samples;

    // normal distrubution to represent encoder noise
    static std::normal_distribution<float> rndmEnc;
    static std::normal_distribution<float> rndmGyr;
    // mersenne twister random number generator
    static std::mt19937 randgen;

    // max samples
    static uint J;

    // odom stuff
    static double prevLE; // create previous encoder value left
    static double prevRE; // create previous encoder value right
    static double prevBE; // create previous encoder value right
    static double prevAngle;
    static pose prevRobot;
    static pose MCLRobot;

    



public:    
    static void init(pose start, int max_samples);
    static void resample();
    static void update();
    static float distanceSensorWeight(sample particle, float d, pose offset);
    static void normalizeSamples();
    static pose getPose();
    static void thread();
    static void renderDistanceSensors(SDL_Renderer* renderer);
    static void renderParticles(SDL_Renderer* renderer);
    static void renderRobot(SDL_Renderer* renderer);
};