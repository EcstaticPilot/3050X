#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor F2 = motor(PORT15, ratio6_1, false);
digital_out Injector = digital_out(Brain.ThreeWirePort.A);
motor LF = motor(PORT18, ratio18_1, false);
motor LB = motor(PORT12, ratio18_1, true);
motor RF = motor(PORT20, ratio18_1, true);
motor RB = motor(PORT4, ratio18_1, false);
motor Intake1 = motor(PORT1, ratio18_1, true);
motor turret = motor(PORT21, ratio18_1, false);
inertial gyro1 = inertial(PORT11);
rotation RotationL = rotation(PORT5, true);
rotation RotationB = rotation(PORT3, true);
inertial turretG = inertial(PORT14);
optical Color = optical(PORT7);
rotation TurretE = rotation(PORT17, false);
optical turretOptical = optical(PORT9);
motor roller = motor(PORT19, ratio36_1, false);

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