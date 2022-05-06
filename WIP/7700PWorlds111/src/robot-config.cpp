#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Lift = motor(PORT1, ratio18_1, true);
motor RFDrive = motor(PORT16, ratio18_1, false);
motor RMDrive = motor(PORT18, ratio18_1, false);
motor RBDrive = motor(PORT19, ratio18_1, false);
motor LFDrive = motor(PORT11, ratio18_1, true);
motor LMDrive = motor(PORT13, ratio18_1, true);
motor LBDrive = motor(PORT12, ratio18_1, true);
controller Controller1 = controller(primary);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
digital_out Backlift = digital_out(Brain.ThreeWirePort.B);
inertial Gyro = inertial(PORT14);
motor ringIntake = motor(PORT2, ratio18_1, true);

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