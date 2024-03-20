#include "vex.h"
#include "sylib/sylib.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor LF = motor(PORT10, ratio6_1, true);  //
motor LM = motor(PORT9, ratio6_1, true);  //
motor LB = motor(PORT8, ratio6_1, true); //

motor RF = motor(PORT1, ratio6_1, false); //
motor RB = motor(PORT4, ratio6_1, false); //
motor RM = motor(PORT3, ratio6_1, false);  //

motor lift = motor(PORT7, ratio18_1, true); 

motor intake = motor(PORT11, ratio18_1, true); //

motor cata = motor(PORT13, ratio18_1, false); //
inertial gyro1 = inertial(PORT6);
//rotation cataRot = rotation(PORT10);

/*
rotation RotationL = rotation(PORT17, false);
rotation RotationR = rotation(PORT5, false);
*/
//rotation RotationB = rotation(PORT6, true);

triport expander = triport(PORT12);
pneumatics wingL = pneumatics(expander.F); //
pneumatics wingR = pneumatics(expander.H);

pneumatics wingLD = pneumatics(expander.D); 
pneumatics wingRD = pneumatics(expander.E);


vex::vision::signature TRIBALL = vex::vision::signature (1, -6729, -5253, -5991, -7657, -5539, -6598, 2.5, 0);

vision camera = vision(PORT5,55,TRIBALL);
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