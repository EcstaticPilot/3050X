using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor F1;
extern motor F2;
extern digital_out Injector;
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern motor Intake1;
extern motor Roller;
extern inertial Gyro;
extern digital_out Endgame1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );