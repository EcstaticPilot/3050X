#include "vex.h"
#include "sylib/sylib.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor LF = motor(PORT14, ratio6_1, true); //
motor LM = motor(PORT3, ratio6_1, true);  //
motor LB = motor(PORT16, ratio6_1, true); //

motor RF = motor(PORT18, ratio6_1, false); //
motor RM = motor(PORT10, ratio6_1, false); //
motor RB = motor(PORT20, ratio6_1, false); //

motor intakeMotor = motor(PORT9, ratio18_1, true); //

motor cataMotorA = motor(PORT15, ratio36_1, true);
motor cataMotorB = motor(PORT17, ratio18_1, false);
motor_group cata = motor_group(cataMotorA, cataMotorB);
inertial gyro1 = inertial(PORT12);

pneumatics wingL = pneumatics(Brain.ThreeWirePort.C); //
pneumatics wingR = pneumatics(Brain.ThreeWirePort.E);

pneumatics wingFL = pneumatics(Brain.ThreeWirePort.B); //
pneumatics wingFR = pneumatics(Brain.ThreeWirePort.A);

pneumatics lift2 = pneumatics(Brain.ThreeWirePort.G);
pneumatics lift1 = pneumatics(Brain.ThreeWirePort.D);


optical Optical = optical(PORT7);

vex::vision::signature TRIBALL = vex::vision::signature(1, -6729, -5253, -5991, -7657, -5539, -6598, 2.5, 0);

vex::vision camera = vision(PORT5, 55, TRIBALL);
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