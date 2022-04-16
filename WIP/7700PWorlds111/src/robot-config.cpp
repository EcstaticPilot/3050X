#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Lift = motor(PORT5, ratio18_1, false);
motor RFDrive = motor(PORT10, ratio18_1, false);
motor RMDrive = motor(PORT9, ratio18_1, false);
motor RBDrive = motor(PORT8, ratio18_1, false);
motor LFDrive = motor(PORT1, ratio18_1, true);
motor LMDrive = motor(PORT2, ratio18_1, true);
motor LBDrive = motor(PORT3, ratio18_1, true);
controller Controller1 = controller(primary);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
digital_out tilter = digital_out(Brain.ThreeWirePort.B);
inertial Gyro = inertial(PORT7);

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