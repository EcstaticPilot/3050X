using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightBackMotor;
extern motor LeftBackMotor;
extern motor LeftLift;
extern controller Controller1;
extern motor Lfront;
extern motor Rfront10;
extern inertial Inertial4;
extern motor RightLift;
extern digital_out claw;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );