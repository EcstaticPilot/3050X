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
  double Heading = M_PI;
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
  RotationL.setPosition(0, degrees);
  RotationB.setPosition(0, degrees);
  RotationR.setPosition(0, degrees);
  RB.resetPosition();
  LB.resetPosition();
  gyro1.resetHeading();
  Brain.Screen.drawRectangle(0, 0, 480, 240, orange);
  while (true)
  {

    lEncoder = RotationL.position(degrees);
    rEncoder = RotationR.position(degrees);
    bEncoder = RotationB.position(degrees);

    // convert encoder distance into distance traveled
    distL = ((lEncoder - prevLE) * M_PI / 180) * lRad;
    distR = ((rEncoder - prevRE) * M_PI / 180) * rRad;
    distB = ((bEncoder - prevBE) * M_PI / 180) * bRad;

    // convert encoder distance into disntance traveled

    prevLE = lEncoder; // create previous encoder value left
    prevRE = rEncoder; // create previous encoder value right
    prevBE = bEncoder; // create previous encoder value back

    Heading = ((gyro1.heading()*M_PI/180));

    deltaHeading = Heading - prevHeading; // calculate change in heading

    prevHeading = Heading;

    if (deltaHeading == 0)
    {
      deltaX = distB;
      deltaY = distL;
    }
    else
    {
      deltaX = 2 * sin(deltaHeading / 2) * ((distB / deltaHeading) + distFromCenterB);
      deltaY = 2 * sin(deltaHeading / 2) * ((distR / deltaHeading) - distFromCenterL);
    }

    averageHeading = prevHeading + (deltaHeading / 2);

    X += (deltaY * sin(averageHeading)) + (deltaX * cos(averageHeading));
    Y += (deltaY * cos(averageHeading)) - (deltaX * sin(averageHeading));
    //x is reversed for some reason
    
    while (Heading >= 2 * M_PI)
    {
      Heading -= 2 * M_PI;
    }
    while (Heading < 0)
    {
      Heading += 2 * M_PI;
    }

    if (deltaX != 0 || deltaY != 0)
    {
      //    std::cout << X << "," << Y << std::endl;
    }

    this_thread::sleep_for(10);
  }
  return 1;
}
