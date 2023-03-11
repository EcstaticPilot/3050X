#include "stdio.h"
#include "vex.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>

double TargetSpeed = 0;
void spinFlywheel(double speed) {
  speed = speed * 127; // speed is in pctage so convert to mV 100% = 12000
                       // mV
  F2.spin(forward, speed, voltageUnits::mV);
}

double FWDrive = 0;
double OldError = 0;
double TBHval = 0;
double fwDrive;
float FSPEED;
extern double X;
extern double Y;
double speedOffset=0;
int controlFlywheelSpeed() {
  std::vector<float> moving_avg;
  int init_count = 0;
  double error;
  while (true) {
    if (init_count > 9) {
      double kp=5;
      FSPEED = std::accumulate(moving_avg.begin(), moving_avg.end(), 0.0) / 10.0;
      moving_avg.erase(moving_avg.begin());
      error = TargetSpeed - FSPEED;
      TargetSpeed=0.263*sqrt( ( (125-X)*(125-X) ) + ( (125-Y)*(125-Y) ) )+35+speedOffset;
      if (TargetSpeed <= 0) {
        F2.stop(coast);
      }
      else
      
      spinFlywheel(TargetSpeed*1.2+error*kp);
      this_thread::sleep_for(50);
      FWDrive = fwDrive;
      OldError = error;
    } 
    else {
      init_count++;
    }
    moving_avg.push_back(F2.velocity(pct));
    // std::cout<<F1.velocity(pct) << ","<<
    // FSPEED<<","<<TargetSpeed<<","<<TargetSpeed+kP*error<<std::endl;
  }
  return 1;
}
