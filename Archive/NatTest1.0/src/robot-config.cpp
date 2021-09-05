#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightBackMotor = motor(PORT5, ratio18_1, true);
motor LeftBackMotor = motor(PORT6, ratio18_1, false);
motor Arm = motor(PORT12, ratio18_1, true);
controller Controller1 = controller(primary);
motor Lintake = motor(PORT16, ratio18_1, true);
motor Rintake = motor(PORT18, ratio18_1, false);
motor Lfront = motor(PORT1, ratio18_1, false);
motor Rfront10 = motor(PORT10, ratio18_1, true);
inertial Inertial4 = inertial(PORT4);
motor Arm2 = motor(PORT20, ratio18_1, false);
digital_out claw = digital_out(Brain.ThreeWirePort.A);

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