#pragma once
using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern digital_out Injector;
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern motor RM;
extern motor LM;
extern optical Optical;
extern motor_group cata;

extern inertial gyro1;

extern rotation RotationB;
extern pneumatics wingL;
extern pneumatics wingR;
extern pneumatics wingFL;
extern pneumatics wingFR;
extern pneumatics lift2;
extern pneumatics lift1;
extern motor intakeMotor;
extern vision camera;

extern vex::vision::signature TRIBALL;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );