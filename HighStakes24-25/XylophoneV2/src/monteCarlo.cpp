#include "vex.h"
#include "monteCarlo.h"
#include <random>
#include <odometry.h>

/**
 * @brief gaussian PDF centered at mean = 0, translated up by 1
 *
 * @param x the x value
 * @param a maximum deviation from the mean before the value goes to aproximately 1
 * @param b maximum output from the function at x=mean
 * @param mean optional, defaults to 0
 * @return float weight ranging from 1 to b
 */
float gaussianWeight(float x, float a, float b, float mean = 0)
{
    return (b - 0.9) * exp(-1 * pow(2 * ((x - mean) / a), 2)) + 0.1;
}

int sign(int a)
{
    return (a > 0 ? 1 : -1);
}
void pose::print()
{
    std::cout << this->x << ", " << this->y << ", " << this->theta << std::endl;
}
// intiialize variables
std::vector<sample> MonteCarlo::samples;
std::normal_distribution<float> MonteCarlo::rndmEnc;
std::normal_distribution<float> MonteCarlo::rndmGyr;
std::mt19937 MonteCarlo::randgen;
uint MonteCarlo::J;
double MonteCarlo::prevLE = 0;
double MonteCarlo::prevRE = 0;
double MonteCarlo::prevBE = 0;
double MonteCarlo::prevAngle = 0;
/**
 * @brief initialize mcl
 *
 * @param start
 * @param max_samples
 */
void MonteCarlo::init(pose start, int max_samples)
{
    // random number generater
    randgen = std::mt19937{Brain.Battery.capacity() + (uint)std::chrono::high_resolution_clock::now().time_since_epoch().count()};

    // nomral distrubution generator for encoder noise
    // TODO: tune
    rndmEnc = std::normal_distribution<float>{0, 0.1};
    rndmGyr = std::normal_distribution<float>{0, 0.2};
    J = max_samples;
    samples.reserve(J);
    std::normal_distribution<float> rdn{0, 5};
    float rx = start.x + rdn(randgen);
    float ry = start.y + rdn(randgen);
    float rt = start.theta + rdn(randgen);
    std::cout << "rndm" << rx << "," << ry << "," << rt << std::endl;
    for (int i = 0; i < J; i++)
    {
        float rx = start.x + rdn(randgen);
        float ry = start.y + rdn(randgen);
        float rt = start.theta + rdn(randgen);
        samples[i] = sample(rx, ry, rt, 1.0f / J);
    }
}

/**
 * @brief update MCL
 *
 */
void MonteCarlo::update()
{
    pose clampoffset = pose(0, -4, 180);
    pose leftoffset = pose(-4.5, -1.5, 90);
    pose rightoffset = pose(4.5, -1.5, -90);
    // update change in robot
    float angle = gyro1.rotation();
    float dAngle = angle - prevAngle;
    // encoders
    float lEncoder = LF.position(degrees) * 3 / 4;
    float rEncoder = RF.position(degrees) * 3 / 4;
    float bEncoder = (rotationB.installed() ? rotationB.position(degrees) : 0);

    float dLE = lEncoder - prevLE;
    float dRE = rEncoder - prevRE;
    float dBE = bEncoder - prevBE;

    prevBE = bEncoder;
    prevLE = lEncoder;
    prevRE = rEncoder;
    bool gpsEnable = false;
    point gpsPT;
    if (GPS.installed() && GPS.quality() == 100)
    {
        gpsEnable = true;
        gpsPT = point(GPS.xPosition(inches), GPS.yPosition(inches));
    }
    // for every particle
    for (int i = 0; i < J; i++)
    {

        // generate random poses
        sample start = samples[i];

        // generate random encoder deltas
        float rLE = dLE + rndmEnc(randgen);
        float rRE = dRE + rndmEnc(randgen);
        float rBE = dBE + rndmEnc(randgen);

        // generate random angle delta
        float rDAngle = dAngle; //+ rndmGyr(randgen);
        float rAngle = prevAngle + rDAngle;

        // get pose from random values
        pose rPose = Odometry::odomStep(rLE, rRE, rBE, rAngle, rDAngle);

        // create sample
        sample rSample = sample(
            start.x + rPose.x,
            start.y + rPose.y,
            rPose.theta,
            1.0f / J);
        // put sample back
        samples[i] = rSample;

        // now to reweight

        // GPS reweight
        if (gpsEnable)
        {
            float d = gpsPT.distance(samples[i]);
            samples[i].w = samples[i].w * gaussianWeight(d, 24, 4);
        }
        // if(i==1){
        //     std::cout<<"rGPS";
        //     samples[i].print();
        // }
        if (clamp.value() == 0 && clampDistance.objectSize() != sizeType::none and false)
        {
            // std::cout<<"clamp"<<std::endl;
            //  pose of current point
            sample tempRobot = samples[i];

            // distance sensor distance
            float d = clampDistance.objectDistance(inches);

            float weight = distanceSensorWeight(tempRobot, d, clampoffset);
            samples[i].w *= weight;
        }
        if (leftDistance.installed() && leftDistance.objectSize() != sizeType::none)
        {
            // std::cout<<"left"<<std::endl;
            //  pose of current point
            sample tempRobot = samples[i];

            // distance sensor distance
            // Cordy was here
            float d = leftDistance.objectDistance(inches);

            float weight = distanceSensorWeight(tempRobot, d, leftoffset);
            samples[i].w *= weight;
        }
        if (rightDistance.installed() && rightDistance.objectSize() != sizeType::none)
        {
            // std::cout<<"right"<<std::endl;
            //  pose of current point
            sample tempRobot = samples[i];

            // distance sensor distance
            // Cordy was here
            float d = rightDistance.objectDistance(inches);

            float weight = distanceSensorWeight(tempRobot, d, rightoffset);
            samples[i].w *= weight;
        }

        // weight will adjust based off distance it should have been - distance it is
    }
    // std::cout<<"weight-"<<samples[1].w<< "   "<<1.0f/J<<std::endl;
    prevAngle = angle;
}
/**
 * @brief calculate weight based on a distance sensor reading
 *
 * @param particle the particle to be evaluated
 * @param d the distance sensor reading
 * @param offset the offset for the distance sensor, containing x,y, and theta
 * @return float
 */
float MonteCarlo::distanceSensorWeight(sample particle, float d, pose offset)
{
    // should be a local only change
    particle.theta += offset.theta;
    particle.theta = std::remainder(offset.theta, 180);
    // convert to standard angle
    float thetaRad = (90 - particle.theta) * M_PI / 180;

    // precompute sin and cos
    float sinTheta = sin(thetaRad);
    float cosTheta = cos(thetaRad);

    // size of field box
    float c = 70;

    // calculate offset
    point offsetdelta = point(
        offset.y * sinTheta + offset.x * cosTheta,
        offset.y * cosTheta - offset.x * sinTheta);

    // distance sensor point, estimate the point where the distance sensor hit
    // added sensor offset from robot position
    point dspt = point(particle.x + d * cosTheta, particle.y + d * sinTheta) + offsetdelta;

    // get distances to
    float d1 = (sinTheta != 0) ? fabs(sign(sinTheta) * c - particle.y) / sinTheta : std::numeric_limits<float>::max();
    float d2 = (cosTheta != 0) ? fabs(sign(cosTheta) * c - particle.x) / cosTheta : std::numeric_limits<float>::max();
    if (sinTheta < 0.00001)
    {
        d1 = 200;
        d2 = fabs(sign(cosTheta) * c - particle.x);
    }
    else if (cosTheta < 0.00001)
    {
        d1 = fabs(sign(sinTheta) * c - particle.y);
        d2 = 200;
    }
    else
    {
        d1 = fabs(sign(sinTheta) * c - particle.y);
        d2 = fabs(sign(cosTheta) * c - particle.x);
    }
    float d3 = dspt.distance(point(0, 24));
    float d4 = dspt.distance(point(24, 0));
    float d5 = dspt.distance(point(0, -24));
    float d6 = dspt.distance(point(-24, 0));

    // distance final//decide between the 4 ladders or wall
    float df = d1;
    if (d2 < df)
        df = d2;
    if (d3 < df)
        df = d3;
    if (d4 < df)
        df = d4;
    if (d5 < df)
        df = d5;
    if (d6 < df)
        df = d6;

    float w = gaussianWeight(df, 2, 100);
    return w;
}
/**
 * @brief resample MCL using stochastic universal resampling
 *
 */
void MonteCarlo::resample()
{
    // stochastic universal resampling
    std::vector<sample> temp{J};

    float start = (float)(((float)randgen() / (float)randgen.max()) / (float)J);

    float sum = samples[0].w;
    float j = start;

    // k is pointer
    float k = 0;

    // std::cout<<"start "<<start<<std::endl;
    //  repeat J times to fill entire space

    for (int i = 0; i < J; i++)
    {
        j = start + (float)((float)(i) / J);
        // while the weight we want is less than current sum, incremenet sum and counter
        // std::cout<<j<<std::endl;
        // std::cout<<"sum"<<sum<<std::endl;
        while (j > sum)
        {
            k++;
            if (k > J - 1)
                break;
            sum += samples[k].w;
            // std::cout<<"sum "<<sum<< " j "<<j<<std::endl;
            //! j is going to a very large integer
        }
        // once we are less than sum, add to new sample
        if (k > J - 1)
        {
            j = 0;
            k = 0;
        }

        temp[i] = samples[k];
        temp[i].w = 1.0f / J;
    }
    samples = temp;
}

/**
 * @brief get current robot pose from MCL
 *
 * @return pose
 */
pose MonteCarlo::getPose()
{
    pose output{0, 0, 0};
    for (sample j : samples)
    {
        output.x += j.x * j.w;
        output.y += j.y * j.w;
        output.theta += j.theta * j.w;
    }
    return output;
}
/**
 * @brief normalize MCL samples so sum of w is 1
 *
 */
void MonteCarlo::normalizeSamples()
{
    float sum = 0;
    for (sample j : samples)
    {
        sum += j.w;
    }
    // evertyhing is ok
    if (sum == 1)
        return;

    // normalize everything
    for (sample j : samples)
    {
        j.w = j.w / sum;
    }
    return;
}
/**
 * @brief main MCL thread
 *
 */
void MonteCarlo::thread()
{
    wait(1, sec);
    robot = point(-60, -36);
    gyro1.setRotation(90, degrees);
    prevAngle = 90;
    MonteCarlo::init(pose(robot, gyro1.rotation()), 1000);
    while (true)
    {
        //double start = Brain.timer(timeUnits::msec);
        MonteCarlo::update();
        MonteCarlo::normalizeSamples();
        MonteCarlo::resample();
        pose output = MonteCarlo::getPose();
        //double end = Brain.timer(timeUnits::msec);
        //std::cout << "dtime" << end - start << std::endl;
        //output.print();
        if(Controller1.ButtonY.pressing()){
            for(sample s:samples){
                s.print();
                wait(5,msec);
            }
        }
        robot.x = output.x;
        robot.y = output.y;
        wait(10, msec);
    }
}