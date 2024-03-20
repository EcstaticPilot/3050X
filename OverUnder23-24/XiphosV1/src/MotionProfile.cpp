#include "MotionProfile.h"
#include "math.h"
/**
 * @brief Construct a new Trapezoidal Profile:: Trapezoidal Profile object
 * 
 * @param target 
 * @param a1 
 * @param a2 
 * @param vmax 
 */
TrapezoidalProfile::TrapezoidalProfile(float target, float a1, float a2, float vmax) : vmax(vmax), a1(a1), a2(a2), target(target)
{
    d1 = a1 / 2 * pow(vmax / a1, 2);
    d3 = target;
    d2 = d3 - (pow(vmax, 2) / (2 * a2));
    shortDist = (d2<d1);
}
// functions for time as a function of position
float TrapezoidalProfile::p1(float x)
{
    float output = sqrt((2 * x) / a1);
    return output;
}
float TrapezoidalProfile::p2(float x)
{
    float output = (x + (pow(vmax, 2) / (2 * a1))) / vmax;
    return output;
}
float TrapezoidalProfile::p3(float x)
{
    float output = -1 * sqrt((-2 * (x - d3)) / a2) + (d3 - ((pow(vmax, 2) - (2 * vmax)) / (2 * a1)) - ((pow(vmax, 2) - (2 * vmax)) / (2 * a2))) / vmax + (vmax / a1) + (vmax / a2);
    return output;
}
/**
 * @brief gets the output
 * 
 * @param dist 
 * @return float velcoity
 */
float TrapezoidalProfile::getOutput(float dist)
{
    float output;
    if(shortDist && dist<d3){
        output = std::min(a1 * p1(dist), -a2 * (p3(dist) - p3(d2)) + vmax);
    }
    else if (dist < d1)
    {
        output = a1 * p1(dist);
    }
    else if (dist < d2)
    {
        output = vmax;
    }
    else if (dist < d3)
    {
        output = -a2 * (p3(dist) - p3(d2)) + vmax;
    }
    else
    {
        output = 0;
    }
    return output;
}