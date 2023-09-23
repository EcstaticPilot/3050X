#include "vex.h"
#include "sylib/sylib.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT3, ratio6_1, true);
motor LM = motor(PORT4, ratio6_1, true);
motor LB = motor(PORT9, ratio6_1, false);

motor RF = motor(PORT1, ratio6_1, false);
motor RM = motor(PORT20, ratio6_1, false);
motor RB = motor(PORT10, ratio6_1, true);

motor cata = motor(PORT19,ratio18_1,true);
inertial gyro1 = inertial(PORT19);
/*
rotation RotationL = rotation(PORT17, false);
rotation RotationR = rotation(PORT5, false);
rotation RotationB = rotation(PORT10, true);
*/
pneumatics wingL = pneumatics(Brain.ThreeWirePort.A);
pneumatics wingR = pneumatics(Brain.ThreeWirePort.B);

pneumatics blockerL = pneumatics(Brain.ThreeWirePort.C);
pneumatics blockerR = pneumatics(Brain.ThreeWirePort.D);

sylib::Addrled LED = sylib::Addrled(21, 1, 64);
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