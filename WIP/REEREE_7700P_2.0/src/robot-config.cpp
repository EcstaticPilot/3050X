#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LFDrive = motor(PORT2, ratio18_1, false);
motor LBDrive = motor(PORT1, ratio18_1, false);
motor LUDrive = motor(PORT3, ratio18_1, true);
motor RFDrive = motor(PORT9, ratio18_1, true);
motor RBDrive = motor(PORT10, ratio18_1, true);
motor RUDrive = motor(PORT8, ratio18_1, false);
digital_out FClaw = digital_out(Brain.ThreeWirePort.A);
digital_out FClaw2 = digital_out(Brain.ThreeWirePort.B);
controller Controller1 = controller(primary);
digital_out BClaw = digital_out(Brain.ThreeWirePort.C);
digital_out Clamp = digital_out(Brain.ThreeWirePort.D);
digital_out Clamp2 = digital_out(Brain.ThreeWirePort.E);
motor Lift = motor(PORT7, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}