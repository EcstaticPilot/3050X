using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor F2;
extern digital_out Injector;
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern motor Intake1;
extern motor turret;
extern inertial gyro1;
extern rotation RotationL;
extern rotation RotationB;
extern inertial turretG;
extern optical Color;
extern rotation TurretE;
extern optical turretOptical;
extern motor roller;
extern digital_out expansion;
extern controller Controller2;
extern digital_out expansion_two;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );