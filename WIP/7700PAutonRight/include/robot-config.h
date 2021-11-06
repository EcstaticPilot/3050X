using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RBDrive;
extern motor LBDrive;
extern motor LLift;
extern controller Controller1;
extern motor LFDrive;
extern motor RLift;
extern motor RFDrive;
extern digital_out Claw2;
extern motor ClawSpin;
extern motor mogolift;
extern digital_out Claw;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );