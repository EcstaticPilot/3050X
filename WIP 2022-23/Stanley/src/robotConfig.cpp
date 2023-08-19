#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT4, ratio18_1, true);
motor LB = motor(PORT9, ratio18_1, false);
motor RF = motor(PORT7, ratio18_1, false);
motor RB = motor(PORT2, ratio18_1, true);
inertial gyro1 = inertial(PORT1);
rotation RotationL = rotation(PORT17, false);
rotation RotationR = rotation(PORT5, false);
rotation RotationB = rotation(PORT10, true);
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