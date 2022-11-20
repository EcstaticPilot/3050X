#include "vex.h"

void spinFlywheel(double speed) {
  speed = speed * 120; // speed is in pctage so convert to mV 100% = 12000
                       // mV
  F1.spin(forward, speed, voltageUnits::mV);
  F2.spin(forward, speed, voltageUnits::mV);
}
