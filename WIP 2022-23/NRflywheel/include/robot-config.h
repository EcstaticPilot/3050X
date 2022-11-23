using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;


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
extern controller Controller2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );