// Motion profiling
class TrapezoidalProfile
{
public:
    float target;

    float a1;
    float a2;
    float vmax;
    float d1;
    float d2;
    float d3;

   

    float p1(float x);
    float p2(float x);
    float p3(float x);
    bool shortDist = false;

    TrapezoidalProfile(float target, float a1, float a2, float vmax = 100);
    float getOutput(float dist);
};

class SCurveProfile
{
public:
    float target;
    float a1;
    float a2;
    float j1;
    float j2;
        float v0;
    float vf;

 bool shortDist;
    float vmax;
    float endTime;

    float distStart;
    float distEnd;

    float timeMarks[8];
    float distMarks[8];

    float p1(float t);
    float p2(float t);
    float p3(float t);
    float p4(float t);
    float p5(float t);
    float p6(float t);
    float p7(float t);

    float v1(float t);
    float v2(float t);
    float v3(float t);
    float v4(float t);
    float v5(float t);
    float v6(float t);
    float v7(float t);

    float tp1(float d);
    float tp2(float d);
    float tp3(float d);
    float tp4(float d);
    float tp5(float d);
    float tp6(float d);
    float tp7(float d);

    /**
     * @brief Construct a new SCurveProfile object
     *
     * @param target
     * @param a1
     * @param a2
     * @param j1
     * @param j2
     * @param v0
     * @param vf
     * @param vmax
     */
    SCurveProfile(float target, float a1, float a2, float j1, float j2, float v0, float vf, float vmax = 100);

    float getOutputDist(float d);
    float getOutput(float d);
    float getOutputTime(float t);
};