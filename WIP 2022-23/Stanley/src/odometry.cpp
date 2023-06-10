#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
double X = 0, Y = 0;

bool trackingwheels = false;

int odometery()
{

  double prevHeading = gyro1.rotation();

  double deltaHeading = 0; // change in heading
  double absoluteOrientation = M_PI;
  double lRad;
  double bRad;
  if(trackingwheels){
  lRad = 1.3926;     // radius of tracking wheel
  bRad = 1.3926;
  }
  else{
  lRad = 2;
  bRad = 0;
  }     // radius of tracking wheel
  double lEncoder = 0;      // declaring encoder variable left
  double bEncoder = 0;      // declaring encoder variable back
  double distL = 0;         // distance left encoder has traveled
  double distB = 0;         // distance back encoder has traveled
  double prevLE = lEncoder; // create previous encoder value left
  double prevBE = bEncoder; // create previous encoder value back
  double averageHeading;    //
  double deltaX;
  double deltaY;
  double speed;
  double prevTime=0;
  double time=0;
  Controller1.rumble(".");
  RotationL.resetPosition();
  RotationB.resetPosition();
  RB.resetPosition();
  LB.resetPosition();
  while (1)
  {
    if (trackingwheels)
    {
      lEncoder = RotationL.position(turns) * 360;
      bEncoder = RotationB.position(turns) * 360;
    }
    else
    {
      time=Brain.timer(timeUnits::msec);
     // speed+=gyro1.acceleration(xaxis)*386088582.67717*(time-prevTime); //converts acceleration to velocity in inches per millisecond
      lEncoder = (RB.position(turns) * 360 + LB.position(turns) * 360) / 2;
      bEncoder=0;//speed*(time-prevTime);
    }
    prevTime=time;
    distL = ((lEncoder - prevLE) * M_PI / 180) * lRad;
    // convert encoder distance into distance traveled
    distB = ((bEncoder - prevBE) * M_PI / 180) * bRad;
    // convert encoder distance into disntance traveled

    prevLE = lEncoder; // create previous encoder value left
    prevBE = bEncoder; // create previous encoder value back

    absoluteOrientation = gyro1.rotation() * M_PI / 180.0;
    deltaHeading =
        absoluteOrientation - prevHeading; // calculate change in heading
    averageHeading = prevHeading + (deltaHeading) / 2;
    prevHeading = absoluteOrientation;
    // ax+by
    // cx +dy
    /*  double a =cos(averageHeading);
      double b =-sin(averageHeading);
      double c =sin(averageHeading);
      double d = cos(averageHeading);
      double xVector= -B*deltaHeading-distB;
      double yVector= L*deltaHeading-distL;*/
    deltaX = (distL * sin(averageHeading)) + (distB * cos(averageHeading));
    deltaY = (distL * cos(averageHeading)) - (distB * sin(averageHeading));

    while (absoluteOrientation >= 2 * M_PI)
    {
      absoluteOrientation -= 2 * M_PI;
    }
    while (absoluteOrientation < 0)
    {
      absoluteOrientation += 2 * M_PI;
    }

    X += deltaX;
    Y += deltaY;
  //  std::cout<<speed<<", "<<X<<","<<Y<<std::endl;
    this_thread::sleep_for(10);
  }
  return 1;
}
