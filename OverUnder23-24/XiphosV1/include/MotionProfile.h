//Motion profiling
#include "vex.h"
class TrapezoidalProfile {
    private:
    float vmax;
    float a1;
    float a2;
    float target;

    float d1;
    float d2;
    float d3;

    float p1(float x);
    float p2(float x);
    float p3(float x);
    bool shortDist;
    public:
    TrapezoidalProfile(float target, float a1, float a2, float vmax =100);
    float getOutput(float dist);
};