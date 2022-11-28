

#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
double TargetSpeed=0;
void spinFlywheel(double speed) {
  speed = speed * 120; // speed is in pctage so convert to mV 100% = 12000
                       // mV
  F1.spin(forward, speed, voltageUnits::mV);
  F2.spin(forward, speed, voltageUnits::mV);
}
double FWDrive=0;
double OldError=0;
double TBHval = 0;
double fwDrive;
int controlFlywheelSpeed() {
  while (true) {
    double kP=0.25;
    double kI = .025;

  double speed = F1.velocity(pct);
  double error = TargetSpeed - speed;
      double fwDrive = FWDrive + kI * error;
  std::cout<<error<<std::endl;
  
 // Brain.Screen.printAt(1, 40, " speed = %.2f ", speed);
  // Keep drive between 0 to 100%
  if (fwDrive > 100)
    fwDrive = 100;
  if (fwDrive < 0)
    fwDrive = 0;
  // Check for zero crossing
  if (error * OldError < 0) {
    fwDrive = 0.5 * (fwDrive + TBHval);
    TBHval = fwDrive;
  }

//  Brain.Screen.printAt(180, 40, "fwdrive %.1f  ", fwDrive);
  if(error>10){speed=100;
  }else {
  speed=TargetSpeed;
  }
  spinFlywheel(TargetSpeed);
  wait(10, msec);
  FWDrive = fwDrive;
  OldError = error;

}
  
  return 1;
}
