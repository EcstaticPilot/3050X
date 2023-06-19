#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
/*
 ██████╗  ██████╗   ██████╗  ███╗   ███╗ ███████╗ ████████╗ ███████╗ ██████╗  ██╗   ██╗
██╔═══██╗ ██╔══██╗ ██╔═══██╗ ████╗ ████║ ██╔════╝ ╚══██╔══╝ ██╔════╝ ██╔══██╗ ╚██╗ ██╔╝
██║   ██║ ██║  ██║ ██║   ██║ ██╔████╔██║ █████╗      ██║    █████╗   ██████╔╝  ╚████╔╝
██║   ██║ ██║  ██║ ██║   ██║ ██║╚██╔╝██║ ██╔══╝      ██║    ██╔══╝   ██╔══██╗   ╚██╔╝
╚██████╔╝ ██████╔╝ ╚██████╔╝ ██║ ╚═╝ ██║ ███████╗    ██║    ███████╗ ██║  ██║    ██║
 ╚═════╝  ╚═════╝   ╚═════╝  ╚═╝     ╚═╝ ╚══════╝    ╚═╝    ╚══════╝ ╚═╝  ╚═╝    ╚═╝

*/

double X = 0, Y = 0;

/**
 * @brief odometry
 */
int odometery()
{

  double prevHeading = gyro1.rotation();

  double deltaHeading = 0; // change in heading
  double absoluteOrientation = M_PI;
  double distFromCenterL = 4.2;
  double distFromCenterB = 6;
  double lRad;
  double bRad;
  double rRad;   // radius of tracking wheel
  lRad = 1.3926; // radius of tracking wheel
  bRad = 1.3926;
  rRad = 1.3926;
  // radius of tracking wheel
  double lEncoder = 0;      // declaring encoder variable left
  double bEncoder = 0;      // declaring encoder variable back
  double rEncoder = 0;      // declaring encoder variable right
  double distL = 0;         // distance left encoder has traveled
  double distB = 0;         // distance back encoder has traveled
  double distR = 0;         // distance right encoder has traveled
  double prevLE = lEncoder; // create previous encoder value left
  double prevBE = bEncoder; // create previous encoder value back
  double prevRE = rEncoder; // create previous encoder value right
  double trackingRadius;
  double averageHeading; //
  double deltaX;
  double deltaY;
  Controller1.rumble(".");
  RotationL.resetPosition();
  RotationB.resetPosition();
  RotationR.resetPosition();
  RB.resetPosition();
  LB.resetPosition();
  Brain.Screen.drawRectangle(0, 0, 480, 240, orange);
  while (true)
  {

    lEncoder = RotationL.position(turns) *2*M_PI;
    rEncoder = RotationR.position(turns) *2*M_PI;
    bEncoder = RotationB.position(turns) *2*M_PI;

    // convert encoder distance into distance traveled
    distL = (lEncoder - prevLE) * lRad;
    distR = (rEncoder - prevRE) * rRad;
    distB = (bEncoder - prevBE) * bRad;

    // convert encoder distance into disntance traveled

    prevLE = lEncoder; // create previous encoder value left
    prevRE = rEncoder; // create previous encoder value right
    prevBE = bEncoder; // create previous encoder value back

    absoluteOrientation = gyro1.rotation() * M_PI / 180.0;
    deltaHeading = (distL - distR) / (distFromCenterL + distFromCenterL); // calculate change in heading
    averageHeading = prevHeading + (deltaHeading / 2);
    prevHeading = absoluteOrientation;
    trackingRadius = (distR / deltaHeading) + distFromCenterL; // calculate radius of tracking wheel radius
  //  if (distL == distR)
  //  {
      deltaX = distB;
      deltaY = (distL+distR)/2;
 /*   }
    else
    {
      deltaX = 2 * sin(deltaHeading / 2) * ((distB / deltaHeading) + distFromCenterB);
      deltaY = 2 * sin(deltaHeading / 2) * ((distR / deltaHeading) + distFromCenterL);
    }*/

    X += (deltaY * sin(averageHeading)) + (deltaX * cos(averageHeading));
    Y += (deltaY * cos(averageHeading)) - (deltaX * sin(averageHeading));

    while (absoluteOrientation >= 2 * M_PI)
    {
      absoluteOrientation -= 2 * M_PI;
    }
    while (absoluteOrientation < 0)
    {
      absoluteOrientation += 2 * M_PI;
    }

    if (deltaX != 0 || deltaY != 0)
    {
  //    std::cout << X << "," << Y << std::endl;
    }

    this_thread::sleep_for(10);
  }
  return 1;
}
