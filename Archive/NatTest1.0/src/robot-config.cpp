#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightBackMotor = motor(PORT20, ratio18_1, true);
motor LeftBackMotor = motor(PORT11, ratio18_1, false);
motor LeftLift = motor(PORT12, ratio18_1, true);
controller Controller1 = controller(primary);
motor Lfront = motor(PORT1, ratio18_1, false);
motor Rfront10 = motor(PORT10, ratio18_1, true);
inertial Inertial4 = inertial(PORT4);
motor RightLift = motor(PORT19, ratio18_1, false);
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