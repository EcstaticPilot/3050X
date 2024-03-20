#include "vex.h"
#include "stanley.h"
#include <vector>
using namespace vex;
void flapWings();
void flapLift();
void spinIntake();
void toggleBlocker();
extern competition Competition;
void stopIntake()
{
	intakeMotor.stop(hold);
}
void testAuton()
{
	robot = point(1, -55);
	gyro1.setRotation(-90, degrees);
	intake();

	inchDrive(2, 5);
	Stanley::setPath(std::vector<point>{
		{1.164, -56.714},
		{35, -54},
		{50, -55},
		{52, -35}});
	Stanley::addEvent([]()
					  { stopIntake(); },
					  0.1);
	Stanley::addEvent([]()
					  { wingL.set(true); },
					  0.4);
	Stanley::addEvent([]()
					  { wingL.set(false); },
					  0.8);
	Stanley::run(fast, true);
}

void offensiveZoneAuton()
{

	std::cout << "auton start";
	//! arc drive now works differently maybe fixed needs testing
	gyro1.resetRotation();
	Stanley::setPath(std::vector<point>{{0, 0}, {0, 20}, {-5, 20}, {-10, 40}});
	Stanley::run();
	// arcDrive_length(30, -37.5);
	rotateTo(60, 0.5);
	outtake();
	vex::wait(500, msec);
	intakeMotor.stop(coast);
	//-30,65

	// float angle = atan((-30-robot.x)/(65-robot.y));
	rotateTo(310);
	// turnToObject(TRIBALL,-50,2,15,false);
	// vex::wait(100, msec);

	intake();
	inchDrive(23.5, 1.5, 1);
	inchDrive(-2);
	rotateTo(90);
	flapWings();
	// vex::wait(500, msec);
	outtake();
	inchDrive(31, 1.25, .75);
	intakeMotor.stop(coast);
	flapWings();
	inchDrive(-10);
	rotateTo(-130);
	turnToObject(TRIBALL, -138);
	intake();
	inchDrive(28);
	vex::wait(100, msec);
	inchDrive(-19);
	rotateTo(90);
	outtake();
	drive(100, 100, 1000);
	intakeMotor.stop(coast);
	inchDrive(-24);
	/*
	//*new one
	Stanley::setPath(std::vector<point>{
		{10.365, -11.969},
		{14.59, -7.48},
		{13.364, 0.598},
		{38.323, 0.598}});
	Stanley::run();
	*/
	/*robot.x = 110;
	robot.y = 10;
	*/
	// rotateTo(90);
	//   code for offensive zone auton

	/*inchDrive(-30);
	rotateTo(45);

	*/
}

void offensiveZoneAuton2()
{
	Stanley::setTimeout(10000);
	std::cout << "run offsence auto 2";
	intake();
	robot = point(35.61, -49.836);
	Stanley::setPath(std::vector<point>{
		{36.61, -49.836},
		{31.848, -22.59},
		{25.913, -11.745},
		{25, -2}}); // 26.442,-1
	Stanley::addEvent([]()
					  { wingFR.set(true); },
					  0.025);
	Stanley::addEvent([]()
					  { wingFR.set(false); },
					  0.1);
	Stanley::run(fast);
	drive_brake(brake);
	vex::wait(100, msec);
	// Stanley::addEvent(stopIntake, 0.25);
	Stanley::setPath(std::vector<point>{
		{24.442, -0.106},
		{23.384, -11.745},
		{36.494, -40.842},
		{37.023, -55.83}});
	Stanley::run(fast, true);
	rotateTo(35, 5);
	outtake();
	vex::wait(200, msec);
	rotateTo(-90, 1);
	// intake.spin(forward, 12, volt);
	// Stanley::setPath(std::vector<point>{
	// 	{robot.x,-55},
	// 	{robot.x+5,-55},
	// 	{robot.x+5,-55},
	// 	{3,-55}
	// });
	// drive_brake();
	// vex::wait(100, msec);
	// Stanley::addEvent(stopIntake, 0.25);

	Stanley::setPath(std::vector<point>{
		{39, -55},
		{45, -54},
		{54, -55},
		{54, -35}});
	Stanley::addEvent([]()
					  { wingL.set(true); },
					  0.2);
	Stanley::addEvent([]()
					  { wingL.set(false); },
					  0.8);
	// Stanley::addEvent([]()
	// 				  { wingL.set(false); },0.8);

	Stanley::run(fast, true);

	// rotateTo(180);
	drive_brake(brake);

	inchDrive(-12, 10, 0.75);

	inchDrive(8, 1.5, 1, 1);
	rotateTo(200);
	inchDrive(-12, 10, 0.75);
	inchDrive(5, 1.5, 1);
	wingL.set(false);
	// rotateTo(180);
	// inchDrive(-10,3,0.5);
	// inchDrive(8);
	rotateTo(-90, 5);
	intake();
	std::cout << robot.x << ", " << robot.y << std::endl;

	Stanley::setPath(std::vector<point>{
		{67, -25},
		{20, -25},
		{30.526, -20},
		{11, -14}});
	Stanley::run(fast);
	drive_brake();
	wait(100, msec);
	rotateTo(80, 10);
	intake();
	// wait(100,msec);
	// rotateTo(0);
	// intake.spin(forward,12,volt);
	// Stanley::setPath(std::vector<point>{
	// 	{10, -15},
	// 	{10, 13.3},
	// 	{5.6, 7.6},
	// 	{42, 2}});
	// 	Stanley::addEvent([](){
	// 		intake.spin(forward,-12,volt);
	// 		wingFL.open();
	// 		wingFR.open();
	// 	},0.75);
	// 	Stanley::run(fast);
	outtake();
	inchDrive(50, 2, 1);
	inchDrive(-12);

	rotateTo(-70);
	intake();
	inchDrive(20);
	rotateTo(90);
	outtake();
	inchDrive(40);
	return;
}
void defensiveZoneAuton()
{
	// code for defensive zone auton
	inchDrive(4, 2);
	wingL.set(true);
	vex::wait(100, msec);
	drive(-50, -25, 500);
	wingL.set(false);
	rotateTo(-45);
	inchDrive(17.6);
	rotateTo(-10);
	outtake();
	vex::wait(250, msec);
	rotateTo(170);
	inchDrive(-15, 2, 1);
	inchDrive(10);
	rotateTo(135);
	inchDrive(27, 0.75);
	rotateTo(91, 0.25);
	vex::wait(500, msec);
	inchDrive(24); // funny number
	drive(10, 10, 0);
	waitUntil(Optical.isNearObject());
	drive_brake(brake);
	std::cout << robot.x << std::endl;
}

void defensiveZoneAuton2()
{
	// fake awp
	robot = point(-51.415, -60.596);
	outtake();
	wait(300, msec);
	intake();
	Stanley::setPath(std::vector<point>{
		{-51.415, -60.596},
		{-40.41, -41.731},
		{-31, -30.53},
		{-31, -17}});
	Stanley::run(fast);
	drive_brake();
	wait(300, msec);
	wingFL.open();
	inchDrive(-12);
	rotateTo(90);
	wingFL.open();
	drive(75,75,0);
	waitUntil(gyro1.pitch(degrees)>10);
	wingFL.close();
	Stanley::setPath(std::vector<point>{
		{-3.466, -6.359},
		{-22.331, -5.77},
		{-72, -70},
		{-42.375, -58.434}});
	Stanley::addEvent([]()
					  { wingL.open(); },
					  0.8);
	Stanley::run(meduim, true);
	wingL.close();
	rotateTo(-31.719660700563697 + 180);
	inchDrive(-33.266,2,1);
	Stanley::setPath(std::vector<point>{
		{-59.865, -30.137},
		{-52.201, -45.661},
		{-41.393, -59.613},
		{-10.737, -59.81}});
	Stanley::run(meduim);
	outtake();
	inchDrive(-40,1,1.5);
	rotateTo(120);
	inchDrive(-10);
}
void disableWatcher()
{
	waitUntil(Competition.isEnabled());
	while (true)
	{
		if (Competition.isEnabled() == false)
		{
			lift1.close();
			lift2.close();
		}
		wait(100, msec);
	}
}
void driverSkillsAuton()
{
	drive(-50, -100, 1000);
	inchDrive(12, 2);

	// align
	rotateTo(70);
	drive(-100, -100, 250);
	inchDrive(3);
	rotateTo(70);
	// touch mathcload bar
	wingR.set(true);
	Controller1.rumble(rumbleShort);
	cata.spin(fwd, 100, rpm);
}
void skillsAuton()
{
	thread hang(disableWatcher);
	Stanley::setTimeout(2);
	// parralelle to bar, as far away as possible
	robot = point(-34.811, -60.946);
	// push in 2 triballs
	drive(-50, -100, 1000);
	inchDrive(12, 2);
	rotateTo(45,3);

	drive(-50, -50, 500);
	// align
	rotateTo(70);
	// touch mathcload bar
	wingR.set(true);
	cata.spin(fwd, 100, rpm);
	//! MAKE THIS NOT ZERO
	wait(30, sec);
	cata.stop(coast);
	outtake();
	wingR.set(false);
	
	// sweep across middle of field
	Stanley::setPath(std::vector<point>{
		{-64.438, -42.826},
		{-46.45, -42.826},
		{-13.914, -43.355},
		{-10.211, -30.923},
		{-10.211, -30.923},
		{-6.507, -18.49},
		{-7.301, 20.659},
		{-7, 35.06}});
	Stanley::addEvent([]()
					  { wingFL.set(true); },
					  0.25);
	Stanley::run(fast);

	std::cout << Stanley::bezierCount << std::endl;
	Stanley::run(fast);
	wingFL.set(false);
	inchDrive(10, 2, 0.5);
	inchDrive(-12);
	// bowl down alley
	Stanley::setPath(std::vector<point>{
		{-9.152, 37.06},
		{-33.885, 33.005},
		{-46.318, 39.883},
		{-39.705, 51.786},
		{-39.705, 51.786},
		{-33.092, 63.69},
		{8.571, 59.544},
		{33.965, 58.75}});
	Stanley::run(fast);
	wingFR.close();
	wingFL.close();
	rotateTo(-90,2);
	// push from side
	wingL.open();
	Stanley::setPath(std::vector<point>{
		{33.965, 58.75},
		{50.894, 58.486},
		{60.417, 44.731},
		{60.681, 35.472}});
	Stanley::run(fast, true);
	inchDrive(-12, 2, 1);
	inchDrive(12, 1.5);
	wingL.close();
	// first front push on upper side
	Stanley::setPath(std::vector<point>{
		{60.417, 34.679},
		{36.478, 29.699},
		{13.464, 34.989},
		{14.258, 25.07},
		{14.258, 25.07},
		{13.464, 22.425},
		{20.871, 12.637},
		{38.594, 11.315}});

	Stanley::addEvent([]()
					  { flapWings(); },
					  1);
	Stanley::run(fast, true);
	inchDrive(-12, 2, 1);
	flapWings();
	inchDrive(12 + 16, 1.5);
	rotateTo(180,3);
	// go across field down
	inchDrive(50,1.5,5,3);
	flapWings();
	// second front push from lower side
	Stanley::setPath(std::vector<point>{
		{12.803, -33.39},
		{12.274, -15.799},
		{19.548, -13.418},
		{42.297, -9.979}});
	Stanley::run(fast, true);
	inchDrive(-12, 2, 1);
	inchDrive(12, 1.5);
	flapWings();
	rotateTo(0,2);

	lift1.open();
	lift2.open();
	// side push
	Stanley::setPath(std::vector<point>{
		{34.626, -14.873},
		{34.361, -38.812},
		{52.878, -69.894},
		{65.046, -30.48}});
	Stanley::addEvent(flapWings, 0.5);
	Stanley::addEvent(flapWings, 0.8);
	Stanley::run(fast, true);

	inchDrive(-12, 2, 1);
	inchDrive(12, 1.5);
	// go to elevation bar

	Stanley::setPath(std::vector<point>{
		{59.491, -38.019},
		{59.227, -52.303},
		{46, -59.842},
		{19.548, -59.842}});
	Stanley::run(fast);

	// wait(500, msec);
	drive(100, 100, 1000);
	return;
	gyro1.resetRotation();
	robot = point(0, 0);
	waitUntil(!gyro1.isCalibrating());
	robot = point(0, 0);
	inchDrive(25);
	rotateTo(80);
	inchDrive(-25, 1, 1);
	// rotateTo(60);

	rotateTo(70);
	wingR.set(true);

	Controller1.rumble("-.-.-");
	cata.spin(fwd, 100, rpm);
	vex::wait(.35, sec);
	drive_brake(hold);
	drive(-0.1, -0.1, 0);
	float waitTime = 0; // 34; // 32
	// driver control inside auton lol
	// driver control inside auton lol

	vex::wait(waitTime, seconds);
	cata.stop(hold);
	// end of cata reset sequence

	wingR.set(false);
	Controller1.rumble("...");

	rotateTo(90);
	Stanley::setPath(std::vector<point>{{robot.x, robot.y}, {robot.x + 2, robot.y - 7}, {-10, 5}, {75, 8}});
	Stanley::run(fast);
	rotateTo(-110);
	wingL.set(true);
	wingR.set(true);
	float angle = gyro1.rotation(degrees) + 180;
	float mag = 0;
	point p2(robot.x + mag * sin(angle), robot.y + mag * cos(angle));
	Stanley::setPath(std::vector<point>{{robot.x, robot.y}, {p2}, {90, 14}, {92.9, 21.8}});
	Stanley::run(meduim, true);
	wingL.set(false);
	wingR.set(false);
	rotateTo(200);
	inchDrive(-18, 2, 1);
	inchDrive(10, 1, 1);
	// rotateTo(190);
	inchDrive(-12, 2, 1);
	inchDrive(10, 1, 1);
	rotateTo(-90);
	Stanley::setPath(std::vector<point>{{robot.x, robot.y}, {robot.x - 36, robot.y + 5}, {55, 30}, {55, 55}});
	Stanley::run(meduim);
	rotateTo(-110);
	wingL.set(true);
	wingR.set(true);
	inchDrive(-33, 2, 1);
	inchDrive(30);
	rotateTo(-110);
	inchDrive(-40, 2, 1);
}
