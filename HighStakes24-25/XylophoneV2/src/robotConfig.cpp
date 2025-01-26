#include "vex.h"
#include "sylib/sylib.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor LF = motor(PORT20, ratio6_1, true); //
motor LM = motor(PORT17, ratio6_1, true);  //
motor LB = motor(PORT4, ratio6_1, true); //

motor RF = motor(PORT5, ratio6_1, false); 
motor RM = motor(PORT10, ratio6_1, false); 
motor RB = motor(PORT9, ratio6_1, false); 

motor intakeMotor1 = motor(PORT6, ratio18_1, false); //
motor intakeMotor2 = motor(PORT19, ratio18_1, false); //
motor_group intake(intakeMotor1,intakeMotor2);
rotation rotationB(PORT8);
inertial gyro1 = inertial(PORT16);
optical clampOptical(PORT3);
pneumatics clamp(Brain.ThreeWirePort.A);
pneumatics armA(Brain.ThreeWirePort.B);
pneumatics armB(Brain.ThreeWirePort.C);
gps GPS(PORT16,-5,-5,inches,90);
vex::vision::signature TRIBALL = vex::vision::signature(1, -6729, -5253, -5991, -7657, -5539, -6598, 2.5, 0);
distance clampDistance(PORT3);
distance leftDistance(PORT1);
distance rightDistance(PORT7);
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