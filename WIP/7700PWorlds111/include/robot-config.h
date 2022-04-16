using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Lift;
extern motor RFDrive;
extern motor RMDrive;
extern motor RBDrive;
extern motor LFDrive;
extern motor LMDrive;
extern motor LBDrive;
extern controller Controller1;
extern digital_out Claw;
extern digital_out tilter;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );