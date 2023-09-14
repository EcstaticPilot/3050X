#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT3, ratio6_1, true);
motor LM = motor(PORT20, ratio6_1, true);
motor LB = motor(PORT9, ratio6_1, true);

motor RF = motor(PORT5, ratio6_1, false);
motor RM = motor(PORT6, ratio6_1, false);
motor RB = motor(PORT10, ratio6_1, false);

motor cata = motor(PORT19,ratio18_1,false);
inertial gyro1 = inertial(PORT13);
/*
rotation RotationL = rotation(PORT17, false);
rotation RotationR = rotation(PORT5, false);
rotation RotationB = rotation(PORT10, true);
*/
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