

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
   float s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11;
  s0=0;
  s1=0;
  s2=0;
  s3=0;
  s4=0;
  s5=0;
  s6=0;
  s7=0;
  s8=0;
  s9=0;
  s10=0;
  s11=0;
  while (true) {
    s11=s10;
    s10=s9;
    s9=s8;
    s8=s7;
    s7=s6;
    s6=s5;
    s5=s4;
    s4=s3;
    s3=s2;
    s2=s1;
    s1=s0;
    s0=F1.velocity(pct);
    FSPEED=(s0+s1+s2+s3+s4+s5+s6+s7+s8+s9+s10+s11)/12;
    double kP=2.5;
double error = TargetSpeed - FSPEED;
     //std::cout<<F1.velocity(pct) << ","<< FSPEED<<","<<TargetSpeed<<","<<TargetSpeed+kP*error<<std::endl;
  spinFlywheel(TargetSpeed+kP*error);
  this_thread::sleep_for(15);
  FWDrive = fwDrive;
  OldError = error;

}
  
  return 1;
}
