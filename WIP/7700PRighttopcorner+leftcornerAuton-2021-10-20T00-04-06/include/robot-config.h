using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RBDrive;
extern motor LBDrive;
extern motor LLift;
extern controller Controller1;
extern motor LFDrive;
extern inertial Gyro;
extern motor RLift;
extern digital_out Claw;
extern motor RFDrive;
extern motor mogolift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );