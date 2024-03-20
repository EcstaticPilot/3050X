#include "vex.h"
#include "sylib/sylib.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor LF = motor(PORT12, ratio6_1, true);  //
motor LM = motor(PORT14, ratio6_1, true);  //
motor LB = motor(PORT13, ratio6_1, true); //

motor RF = motor(PORT11, ratio6_1, false); //
motor RB = motor(PORT16, ratio6_1, false); //
motor RM = motor(PORT18, ratio6_1, false);  //

motor intake = motor(PORT17, ratio18_1, true); //

motor cata = motor(PORT15, ratio18_1, true); //
inertial gyro1 = inertial(PORT2);
rotation cataRot = rotation(PORT10);
/*
rotation RotationL = rotation(PORT17, false);
rotation RotationR = rotation(PORT5, false);
*/
rotation RotationB = rotation(PORT6, true);

triport expander = triport(PORT19);
pneumatics wingL = pneumatics(expander.D); //
pneumatics wingR = pneumatics(expander.C);

pneumatics blockerL = pneumatics(expander.E); //
pneumatics blockerR = pneumatics(expander.G);
vex::vision::signature TRIBALL = vex::vision::signature (1, -6451, -5775, -6113, -6837, -5957, -6397, 11, 0);

vision camera = vision(PORT2,55,TRIBALL);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
  // nothing to initialize
}