#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT6, ratio18_1, true);
motor LB = motor(PORT9, ratio18_1, false);
motor RF = motor(PORT1, ratio18_1, false);
motor RB = motor(PORT2, ratio18_1, true);
inertial gyro1 = inertial(PORT8);
rotation RotationL = rotation(PORT4, true);
rotation RotationR = rotation(PORT5, false);
rotation RotationB = rotation(PORT10, false);
pneumatics claw = pneumatics(Brain.ThreeWirePort.B);


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