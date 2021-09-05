using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LFMotor20;
extern motor LBMotor10;
extern motor RFMotor11;
extern motor RBMotor1;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );