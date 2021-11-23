#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RBDrive = motor(PORT20, ratio18_1, true);
motor LBDrive = motor(PORT11, ratio18_1, false);
motor LLift = motor(PORT2, ratio18_1, true);
controller Controller1 = controller(primary);
motor LFDrive = motor(PORT3, ratio18_1, false);
motor RLift = motor(PORT9, ratio18_1, false);
motor RFDrive = motor(PORT10, ratio18_1, true);
digital_out Claw2 = digital_out(Brain.ThreeWirePort.B);
motor ClawSpin = motor(PORT6, ratio18_1, false);
motor mogolift = motor(PORT8, ratio18_1, true);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
inertial Gyro = inertial(PORT13);

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