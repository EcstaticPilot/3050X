using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LFDrive;
extern motor LBDrive;
extern motor LUDrive;
extern motor RFDrive;
extern motor RBDrive;
extern motor RUDrive;
extern digital_out Claw;
extern digital_out Tilter;
extern motor Lift;
extern inertial Gyro;
extern motor backLift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );