

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
float FSPEED;
int controlFlywheelSpeed() {
   float s0,s1,s2,s3,s4,s5;
  s0=0;
  s1=0;
  s2=0;
  s3=0;
  s4=0;
  s5=0;
  while (true) {
     s5=s4;
    s4=s3;
    s3=s2;
    s2=s1;
    s1=s0;
    s0=F1.velocity(pct);
    FSPEED=(s0+s1+s2+s3+s4+s5)/6;
    double kP=2;
double error = TargetSpeed - FSPEED;
     std::cout<<F1.velocity(pct) << ","<< FSPEED<<","<<TargetSpeed<<","<<TargetSpeed+kP*error<<std::endl;
  spinFlywheel(TargetSpeed+kP*error);
  wait(12, msec);
  FWDrive = fwDrive;
  OldError = error;

}
  
  return 1;
}
