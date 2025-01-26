#include "vex.h"
#include "monteCarlo.h"
#include <random>
#include <odometry.h>
void MonteCarlo::init(pose start, int max_samples)
{
    randgen = std::mt19937{Brain.Battery.capacity()};
    J = max_samples;
    samples.reserve(J);
    for (int i = 0; i < J; i++)
    {
        std::normal_distribution<float> rdn{0, 5};
        float rx = start.x + rdn(randgen);
        float ry = start.y + rdn(randgen);
        float rt = start.theta + rdn(randgen);
        samples[i] = sample(rx, ry, rt, 1.0f / J);
    }
}
void MonteCarlo::update()
{
    //for every particle
    for(int i = 0; i<J;i++){
        
    }
}

void MonteCarlo::resample()
{
    // stochastic universal resampling
    std::vector<sample> temp{J};

    float start = 1 / (randgen() % J);
    float sum = samples[0].w;
    float j = start;
    float k = 0;
    //repeat J times to fill entire space
    for (int i = 0; i < J; i++)
    {
        j=start + (i-1)/J;
        //while the weight we want is less than current sum, incremenet sum and counter
        while (j > sum)
        {
            k++;
            sum += samples[k].w;
        }
        //once we are less than sum, add to new sample
        temp[i] = samples[k];
    }
    samples = temp;
}