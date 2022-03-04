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
extern digital_out FClaw;
extern digital_out FClaw2;
extern digital_out Clamp;
extern digital_out Clamp2;
extern motor Lift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );