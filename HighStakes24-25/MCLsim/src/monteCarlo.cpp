#pragma once
#include <iostream>
#include "monteCarlo.h"
#include <chrono>
#include <random>
#include <odometry.h>
#include <SDL2/SDL.h>
#include <algorithm>
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
    return (b - .001) * exp(-1 * pow(2 * ((x - mean) / a), 2)) + .001;
}

int sign(float a)
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
pose MonteCarlo::prevRobot = pose();
pose MonteCarlo::MCLRobot = pose();
float remap(float value, float a, float b, float c, float d)
{
    return c + ((value - a) / (b - a)) * (d - c);
}
float lineCircleIntersect(pose ray, point c, float r)
{
    // Convert angle to radians
    float thetaRad = (90-ray.theta) * M_PI / 180.0;

    // Calculate the direction vector of the ray
    float rayDirX = cos(thetaRad);
    float rayDirY = sin(thetaRad);

    float m = rayDirY / rayDirX;
    float b = ray.y - m * ray.x;

    float m2 = -1 / m;
    float b2 = c.y - m2 * c.x;

    // calculate intersection point
    float x = (b2 - b) / (m - m2);
    float y = m * x + b;

    float d1 = sqrt(pow(x - c.x, 2) + pow(y - c.y, 2));
    if (d1 > r)
        return -1000;

    if (sign(x - ray.x) != sign(rayDirX) || sign(y - ray.y) != sign(rayDirY))
        return -1000;

    float d2 = sqrt(r * r - d1 * d1);
    point i = point(x + sign(ray.x - x) * fabs(d2 * rayDirX), y + sign(ray.y - y) * fabs(d2 * rayDirY));
    return i.distance(ray);
}
/**
 * @brief initialize mcl
 *
 * @param start
 * @param max_samples
 */
void MonteCarlo::init(pose start, int max_samples)
{
    std::cout << "init" << std::endl;
    // random number generater
    randgen = std::mt19937{(uint)std::chrono::high_resolution_clock::now().time_since_epoch().count()};

    // nomral distrubution generator for encoder noise
    // TODO: tune
    rndmEnc = std::normal_distribution<float>{0, 0.1};
    rndmGyr = std::normal_distribution<float>{0, 0.2};
    J = max_samples;
    samples.reserve(J);
    prevRobot = start;
    std::normal_distribution<float> rdn{0, 5};
    float rx = start.x + rdn(randgen);
    float ry = start.y + rdn(randgen);
    float rt = start.theta + rdn(randgen);
    std::cout << "rndm" << rx << "," << ry << "," << rt << std::endl;
    for (int i = 0; i < J; i++)
    {
        //std::cout<<i<<std::endl;
        float rx = start.x + rdn(randgen);
        float ry = start.y + rdn(randgen);
        float rt = start.theta + rdn(randgen);
        samples[i] = sample(rx, ry, rt, 1.0f / J);
        //samples[i].print();
    }
}
/**
 * @brief perform a raycast to simulate a real distance sensor
 *
 * @param offset
 * @param pos
 * @return float
 */
float distanceReal(pose offset, pose pos)
{
    // should be a local only change
    // std::cout<<"pos"<<pos.theta<<std::endl;

    float thetaRad = (-pos.theta) * M_PI / 180;
    float sinTheta = sin(thetaRad);
    float cosTheta = cos(thetaRad);

    // std::cout<<"theta "<<pos.theta<<std::endl;
    // std::cout<<90-pos.theta<<std::endl;
    // std::cout<<"thetarad"<<thetaRad<<std::endl;

    point offsetdelta = point(
        offset.x * cosTheta - offset.y * sinTheta,
        offset.x * sinTheta + offset.y * cosTheta);
    // offset.print();
    //offsetdelta.print();
    pos.theta += offset.theta;
    // std::cout << "offset" << offsetdelta.x << "," << offsetdelta.y << std::endl;
    //  std::cout<<"offset"<<offset.theta<<std::endl;
    //  pos.theta = std::remainder(offset.theta, 180);
    //   convert to standard angle
    thetaRad = (90 - pos.theta) * M_PI / 180;

    // precompute sin and cos
    sinTheta = sin(thetaRad);
    cosTheta = cos(thetaRad);
    // std::cout<<"sin"<<sinTheta<<std::endl;
    // std::cout<<"cos"<<cosTheta<<std::endl;
    // size of field box
    float c = 70;

    // calculate offset

    // offsetdelta.print();
    pos.x += offsetdelta.x;
    pos.y += offsetdelta.y;
    //std::cout << "offset delta dr: ";
   // offsetdelta.print();
    // get distances to
    float d1 = (sinTheta != 0) ? fabs((sign(sinTheta) * c) - pos.y) / sinTheta : std::numeric_limits<float>::max();
    float d2 = (cosTheta != 0) ? fabs((sign(cosTheta) * c) - pos.x) / cosTheta : std::numeric_limits<float>::max();
    d1 = fabs(d1);
    d2 = fabs(d2);
    // std::cout<<"pos: " << pos.x << "," << pos.y << std::endl;
    // std::cout<<"dx: "<<d2<<std::endl;
    // std::cout<<"dy: "<<d1<<std::endl;
    // std::cout<<"posx: "<<pos.x<<std::endl;
    // std::cout<<"posy: "<<pos.y<<std::endl;

    // std::cout<<"dwx: "<<fabs((sign(cosTheta) * c) - pos.x)<<std::endl;
    // std::cout<<"dwy: "<<fabs((sign(sinTheta) * c) - pos.y)<<std::endl;
    // std::cout<<"sin: "<<sinTheta<<std::endl;
    // std::cout<<"cos: "<<cosTheta<<std::endl;
    // if (sinTheta < 0.00001)
    // {
    //     d1 = 2000;
    //     d2 = fabs(sign(cosTheta) * c - pos.x);
    // }
    // else if (cosTheta < 0.00001)
    // {
    //     d1 = fabs(sign(sinTheta) * c - pos.y);
    //     d2 = 2000;
    // }
    // else
    // {
    //     d1 = fabs(sign(sinTheta) * c - pos.y);
    //     d2 = fabs(sign(cosTheta) * c - pos.x);
    // }
    // calculate whether we hit ladder
   // std::cout << "pos: ";
   // pos.print();
    float d3 = fabs(lineCircleIntersect(pos, point(0, 24), 2));
    float d4 = fabs(lineCircleIntersect(pos, point(24, 0), 2));
    float d5 = fabs(lineCircleIntersect(pos, point(0, -24), 2));
    float d6 = fabs(lineCircleIntersect(pos, point(-24, 0), 2));
    // pick the smallest distance
    float df = d1;
    if (d2 < df)
    {
       // std::cout << "d2: " << d2 << std::endl;
        df = d2;
    }
    if (d3 < df)
    {
       // std::cout << "d3 top: " << d3 << std::endl;
        df = d3;
    }
    if (d4 < df)
    {
       // std::cout << "d4 right: " << d4 << std::endl;
        df = d4;
    }
    if (d5 < df)
    {
      //  std::cout << "d5 bottom: " << d5 << std::endl;
        df = d5;
    }
    if (d6 < df)
    {
      //  std::cout << "d6 left: " << d6 << std::endl;
        df = d6;
    }
    if (df > 78.74)
        df = -1;
    return df;
}
/**
 * @brief update MCL
 *
 */
void MonteCarlo::update()
{
    pose clampoffset = pose(0, -4, 180);
    pose leftoffset = pose(-4.5, -1.5, -90);
    pose rightoffset = pose(4.5, -1.5, 90);
    // // update change in robot
    float angle = Robot.theta ;//+ rndmGyr(randgen);
    float dAngle = angle - prevAngle;

    // get robot pose, add noise
    std::normal_distribution<float> rdn{0, 0.2};
    MCLRobot = {Robot.x + rdn(randgen), Robot.y + rdn(randgen), angle};

    point delta = {MCLRobot.x - prevRobot.x, MCLRobot.y - prevRobot.y};
   // delta.print();
    // // encoders
    // float lEncoder = LF.position(degrees) * 3 / 4;
    // float rEncoder = RF.position(degrees) * 3 / 4;
    // float bEncoder = (rotationB.installed() ? rotationB.position(degrees) : 0);

    // float dLE = lEncoder - prevLE;
    // float dRE = rEncoder - prevRE;
    // float dBE = bEncoder - prevBE;

    // prevBE = bEncoder;
    // prevLE = lEncoder;
    // prevRE = rEncoder;
    // bool gpsEnable = false;
    // point gpsPT;
    // if (GPS.installed() && GPS.quality() == 100)
    // {
    //     gpsEnable = true;
    //     gpsPT = point(GPS.xPosition(inches), GPS.yPosition(inches));
    // }
    // for every particle
    rdn = std::normal_distribution<float>(1, 0.5);
    auto rdn2 = std::normal_distribution<float>(0, 0.4);
    float speed = sqrt(delta.x * delta.x + delta.y * delta.y);
    float sinTheta = sin((90-angle) * M_PI / 180);
    float cosTheta = cos((90-angle) * M_PI / 180);
    std::normal_distribution<float> rdn3{1, 0.025};
    float r1 = rdn3(randgen);
    float r2 = rdn3(randgen);
    float r3 = rdn3(randgen);
    for (int i = 0; i < J; i++)
    {

        // generate random poses
        sample start = samples[i];

        // // generate random encoder deltas
        // float rLE = dLE + rndmEnc(randgen);
        // float rRE = dRE + rndmEnc(randgen);
        // float rBE = dBE + rndmEnc(randgen);

        // // generate random angle delta
        float rDAngle = dAngle + rndmGyr(randgen);
        float rAngle = prevAngle + rDAngle;

        // get pose from random values

        // pose rPose = Odometry::odomStep(rLE, rRE, rBE, rAngle, rDAngle);

        pose rPose = {delta.x + speed*rdn2(randgen),delta.y+speed*rdn2(randgen), rAngle};
        //std::cout<<"speed"<< speed<<std::endl;
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

        // if (gpsEnable)
        // {
        //     float d = gpsPT.distance(samples[i]);
        //     samples[i].w = samples[i].w * gaussianWeight(d, 24, 4);
        // }

        // if(i==1){
        //     std::cout<<"rGPS";
        //     samples[i].print();
        // }
        if (true) // clamp.value() == 0 && clampDistance.objectSize() != sizeType::none and false)
        {
            // std::cout<<"clamp"<<std::endl;
            //  pose of current point
            sample tempRobot = samples[i];

            // distance sensor distance
            float d = distanceReal(clampoffset, Robot);
            float weight = 1;
            if (d > 0)
            {
                weight = distanceSensorWeight(tempRobot, d, clampoffset) *r1;
            }
            samples[i].w *= weight;
        }
        if (true) // leftDistance.installed() && leftDistance.objectSize() != sizeType::none)
        {
            // std::cout<<"left"<<std::endl;
            //  pose of current point
            sample tempRobot = samples[i];

            // distance sensor distance
            // Cordy was here
            float d = distanceReal(leftoffset, Robot);

            float weight = 1;

            if (d > 0)
            {
                weight = distanceSensorWeight(tempRobot, d, leftoffset) * r2;
            }
            if(i==1){
                std::cout<<d<<", "<<weight<<std::endl;
            }
            samples[i].w *= weight;
        }
        if (true) // rightDistance.installed() && rightDistance.objectSize() != sizeType::none)
        {
            // std::cout<<"right"<<std::endl;
            //  pose of current point
            sample tempRobot = samples[i];

            // distance sensor distance
            // Cordy was here
            float d = distanceReal(rightoffset, Robot);
            float weight = 1;
            if (d > 0)
            {
                weight = distanceSensorWeight(tempRobot, d, rightoffset) *r3;
            }
            samples[i].w *= weight;
        }

        // weight will adjust based off distance it should have been - distance it is
    }
    // std::cout<<"weight-"<<samples[1].w<< "   "<<1.0f/J<<std::endl;
    prevAngle = angle;
    prevRobot = MCLRobot;
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
    float df = distanceReal(offset, particle);
    // if(fabs(df-d>12))
    //     return 1;
    float w = gaussianWeight((df-d), 6, 2);
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
    MonteCarlo::normalizeSamples();
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
    // wait(1, sec);
    Robot = pose(-60, -36, 0);
    // gyro1.setRotation(90, degrees);
    prevAngle = 90;
    // MonteCarlo::init(pose(robot, gyro1.rotation()), 1000);
    while (true)
    {
        // double start = Brain.timer(timeUnits::msec);
        MonteCarlo::update();
        MonteCarlo::normalizeSamples();
        MonteCarlo::resample();
        pose output = MonteCarlo::getPose();
        // double end = Brain.timer(timeUnits::msec);
        // std::cout << "dtime" << end - start << std::endl;
        // output.print();
        //  if(Controller1.ButtonY.pressing()){
        //      for(sample s:samples){
        //          s.print();
        //          wait(5,msec);
        //      }
        //  }
        Robot.x = output.x;
        Robot.y = output.y;
        // wait(10, msec);
    }
}

//  RENDERING
extern const double SCALE_FACTOR;
/**
 * @brief render distance sensors
 *
 */
void MonteCarlo::renderDistanceSensors(SDL_Renderer *renderer)
{
    std::normal_distribution<float> rdn{1, 0.025};
    int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
    int c = (1000 - boxSize) / 2;
    pose clampoffset = pose(0, -4, 180);
    pose leftoffset = pose(-4.5, -1.5, -90);
    pose rightoffset = pose(4.5, -1.5, 90);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    // SDL_RenderDrawRect(renderer, new SDL_Rect{0, 0, 900, 900});
    float d = distanceReal(leftoffset, Robot) * rdn(randgen);

    if (d > 0)
    {
        pose offset = leftoffset;
        float sensorTheta = Robot.theta + offset.theta;
        float thetaRad = (90 - sensorTheta) * M_PI / 180;

        // precompute sin and cos
        float sinTheta = sin(thetaRad);
        float cosTheta = cos(thetaRad);

        // float thetaRad2 = (90- Robot.theta) * M_PI / 180;

        // calculate offset
        /*
        point offsetdelta = point(
            offset.y * cos(thetaRad2) - offset.x * sin(thetaRad2),
            offset.y * sin(thetaRad2) + offset.x * cos(thetaRad2));

        std::cout << "offset: " << offsetdelta.x << "," << offsetdelta.y << std::endl;
        */
        float thetaRad2 = (-Robot.theta) * M_PI / 180;
        float sinTheta1 = sin(thetaRad2);
        float cosTheta1 = cos(thetaRad2);

        // std::cout<<"theta "<<pos.theta<<std::endl;
        // std::cout<<90-pos.theta<<std::endl;
        // std::cout<<"thetarad"<<thetaRad<<std::endl;

        point offsetdelta = point(
            offset.x * cosTheta1 - offset.y * sinTheta1,
            offset.x * sinTheta1 + offset.y * cosTheta1);
        // offset.print();
        //offsetdelta.print();
        // in px
        float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
        float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        //*SCALE_FACTOR
        float endX = sensorX + d * SCALE_FACTOR * cosTheta;
        float endY = sensorY - d * SCALE_FACTOR * sinTheta;

        SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX-5, (int)sensorY-5, 10, 10});
        SDL_RenderDrawLine(renderer,
                           sensorX,
                           sensorY,
                           endX,
                           endY);
    }

    d = distanceReal(clampoffset, Robot) * rdn(randgen);

    if (d > 0)
    {
        pose offset = clampoffset;
        float sensorTheta = Robot.theta + offset.theta;
        float thetaRad = (90 - sensorTheta) * M_PI / 180;

        // precompute sin and cos
        float sinTheta = sin(thetaRad);
        float cosTheta = cos(thetaRad);

        float thetaRad2 = (90 - Robot.theta) * M_PI / 180;

        // calculate offset
        point offsetdelta = point(
            offset.y * cos(thetaRad2) - offset.x * sin(thetaRad2),
            offset.y * sin(thetaRad2) + offset.x * cos(thetaRad2));
        //std::cout << "offset" << offsetdelta.x << "," << offsetdelta.y << std::endl;
        // in px
        float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
        float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        //*SCALE_FACTOR
        float endX = sensorX + d * SCALE_FACTOR * cosTheta;
        float endY = sensorY - d * SCALE_FACTOR * sinTheta;

        SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX-5, (int)sensorY-5, 10, 10});
        SDL_RenderDrawLine(renderer,
                           sensorX,
                           sensorY,
                           endX,
                           endY);
    }
    d = distanceReal(rightoffset, Robot)* rdn(randgen);

    if (d > 0)
    {
        pose offset = rightoffset;
        float sensorTheta = Robot.theta + offset.theta;
        float thetaRad = (90 - sensorTheta) * M_PI / 180;

        // precompute sin and cos
        float sinTheta = sin(thetaRad);
        float cosTheta = cos(thetaRad);

        // float thetaRad2 = (90- Robot.theta) * M_PI / 180;

        // calculate offset
        /*
        point offsetdelta = point(
            offset.y * cos(thetaRad2) - offset.x * sin(thetaRad2),
            offset.y * sin(thetaRad2) + offset.x * cos(thetaRad2));

        std::cout << "offset: " << offsetdelta.x << "," << offsetdelta.y << std::endl;
        */
        float thetaRad2 = (-Robot.theta) * M_PI / 180;
        float sinTheta1 = sin(thetaRad2);
        float cosTheta1 = cos(thetaRad2);

        // std::cout<<"theta "<<pos.theta<<std::endl;
        // std::cout<<90-pos.theta<<std::endl;
        // std::cout<<"thetarad"<<thetaRad<<std::endl;

        point offsetdelta = point(
            offset.x * cosTheta1 - offset.y * sinTheta1,
            offset.x * sinTheta1 + offset.y * cosTheta1);
        // offset.print();
        //offsetdelta.print();
        // in px
        float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
        float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        //*SCALE_FACTOR
        float endX = sensorX + d * SCALE_FACTOR * cosTheta;
        float endY = sensorY - d * SCALE_FACTOR * sinTheta;

        SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX-5, (int)sensorY-5, 10, 10});
        SDL_RenderDrawLine(renderer,
                           sensorX,
                           sensorY,
                           endX,
                           endY);
    }
    //std::cout << std::endl;
}

void MonteCarlo::renderParticles(SDL_Renderer *renderer)
{
    int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
    int c = (1000 - boxSize) / 2;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    auto maxElement = std::max_element(samples.begin(), samples.end(),
        [](const sample& a, const sample& b) {
            return a.w < b.w;
        });
    float maxW = maxElement->w;
    //std::cout << "particles" << std::endl;
    for (int i =0; i<J;i++)
    {
        sample j = samples[i];
        //j.print();
        float x = (c + boxSize / 2) + j.x * SCALE_FACTOR;
        float y = (c + boxSize / 2) - j.y * SCALE_FACTOR;
        //SDL_SetRenderDrawColor(renderer, 0, 255, 0,100);
        // if(j.w>1.0f/J){
        //     int a = 255/8 *j.w * J;
        //     SDL_SetRenderDrawColor(renderer, 0, 255, a,100);
        // }
        // else if(j.w<1.0f/J)
        //     SDL_SetRenderDrawColor(renderer, 0, 255, 255,50);
        // else
        //     SDL_SetRenderDrawColor(renderer, 0, 255, 0,50);
        //std::cout << "x: " << x << " y: " << y << std::endl;
        int a = remap(j.w,0,maxW,0,255);
        SDL_SetRenderDrawColor(renderer, a, 255,0,100);
        SDL_FRect rect = {x - 2, y - 2, 4, 4}; // Adjust the size of the rectangle as needed
        SDL_RenderFillRectF(renderer, &rect);
    }
}