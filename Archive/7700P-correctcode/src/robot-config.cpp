#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftBack = motor(PORT11, ratio18_1, false);
motor LeftFront = motor(PORT1, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, true);
motor RightFront = motor(PORT10, ratio18_1, true);
inertial Gyro = inertial(PORT17);
motor RL19 = motor(PORT19, ratio18_1, true);
motor LL12 = motor(PORT12, ratio18_1, false);
digital_out Grabber = digital_out(Brain.ThreeWirePort.A);

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