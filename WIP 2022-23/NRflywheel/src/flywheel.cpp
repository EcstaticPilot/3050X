

#include "stdio.h"
#include "vex.h"
#include <math.h>

double TargetSpeed=0;
void spinFlywheel(double speed) {
  speed = speed * 120; // speed is in pctage so convert to mV 100% = 12000
                       // mV

  F1.spin(fwd, speed, vex::voltageUnits::mV);
  F2.spin(fwd, speed, vex::voltageUnits::mV);
}
double OldError = 0.0;
double TBHval = 0.0;
double FWDrive = 0.0;
int controlFlywheelSpeed() {
  double kI = .025;
  while (true) {
    
        double speed = F1.velocity(pct);
        double error = TargetSpeed - speed;
       double fwDrive = FWDrive + kI * error;
        // :D
        // Brain.Screen.printAt(1, 40, " speed = %.2f ", speed);
        // Keep drive between 0 to 100%
    
        if (error > 20) {
          fwDrive = 100;
        }

        else {
          if (fwDrive > 100)
            fwDrive = 100;
          if (fwDrive <= 0)
            fwDrive = 0;
          // Check for zero crossing
          if (error * OldError < 0) {
            fwDrive = 0.5 * (fwDrive + TBHval);
            TBHval = fwDrive;
          }
        }
     FWDrive = fwDrive;
     OldError = error;
  }
  return 1;
}
