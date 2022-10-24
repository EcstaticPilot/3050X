using namespace vex;

extern brain Brain;

using signature = vision::signature;

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
extern motor turret;
extern inertial gyro1;
extern rotation RotationL;
extern rotation RotationB;
extern inertial turretG;
extern optical Color;
extern rotation TurretE;
extern signature Vision5__SIG_1;
extern signature Vision5__SIG_2;
extern signature Vision5__SIG_3;
extern signature Vision5__SIG_4;
extern signature Vision5__SIG_5;
extern signature Vision5__SIG_6;
extern signature Vision5__SIG_7;
extern vision Vision5;
extern optical turretOptical;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );