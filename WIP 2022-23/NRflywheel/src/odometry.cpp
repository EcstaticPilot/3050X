
#include "stdio.h"
#include "vex.h"

#include <math.h>

double X=0,Y=0;
int odometery() {
double prevHeading = gyro1.heading();
double deltaHeading = 0; // change in heading
double absoluteOrientation = M_PI;
double lRad = 1.375;      // radius of tracking wheel
double bRad = 1.375;      // radius of tracking wheel
double lEncoder = 0;      // declaring encoder variable left
double bEncoder = 0;      // declaring encoder variable back
double distL = 0;         // distance left encoder has traveled
double distB = 0;         // distance back encoder has traveled
double prevLE = lEncoder; // create previous encoder value left
double prevBE = bEncoder; // create previous encoder value back
double averageHeading;    //
double deltaX;
double deltaY;
  Controller1.rumble(".");
  RotationL.resetPosition();
  RotationB.resetPosition();
  while (1) {
    lEncoder = RotationL.position(degrees);
    bEncoder = RotationB.position(degrees);

    distL = ((lEncoder - prevLE) * M_PI / 180) * lRad;
    // convert encoder distance into distance traveled
    distB = ((bEncoder - prevBE) * M_PI / 180) * bRad;
    // convert encoder distance into disntance traveled

    prevLE = lEncoder; // create previous encoder value left
    prevBE = bEncoder; // create previous encoder value back

    absoluteOrientation = gyro1.heading(degrees) * M_PI / 180.0;
    deltaHeading =
        absoluteOrientation - prevHeading; // calculate change in heading
    averageHeading = prevHeading + (deltaHeading) / 2;
    prevHeading = absoluteOrientation;

    deltaX = (distL * sin(averageHeading)) + (distB * cos(averageHeading));
    deltaY = (distL * cos(averageHeading)) - (distB * sin(averageHeading));
    while (absoluteOrientation >= 2 * M_PI) {
      absoluteOrientation -= 2 * M_PI;
    }

    while (absoluteOrientation < 0) {
      absoluteOrientation += 2 * M_PI;
    }
    X += deltaX;
    Y += deltaY;

    this_thread::sleep_for(5);
  }
  return 1;
}
