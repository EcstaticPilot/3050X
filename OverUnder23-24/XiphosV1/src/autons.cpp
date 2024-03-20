#include "vex.h"
#include "stanley.h"
#include <vector>

void flapWings();
void toggleBlocker();

void offensiveZoneAuton()
{

	std::cout << "auton start";
	//! arc drive now works differently maybe fixed needs testing
	gyro1.resetRotation();
	arcDrive_length(50, -37.5);
	rotateTo(80);
	intake.spin(fwd, -12, volt);
	wait(500, msec);
	intake.stop(coast);
	rotateTo(-60);
	wait(100, msec);
	intake.spin(fwd, 100, pct);
	inchDrive(20, 1.5, 1);
	inchDrive(-2);

	rotateTo(90);
	flapWings();
	wait(500, msec);
	intake.spin(reverse, 100, pct);
	inchDrive(31, 1.25, .75);
	intake.stop(coast);
	flapWings();
	inchDrive(-14);
	rotateTo(-138);
	intake.spin(forward, 100, pct);
	inchDrive(27);
	wait(500, msec);
	inchDrive(-19);
	rotateTo(75);
	intake.spin(reverse, 100, pct);
	drive(100, 100, 1000);
	intake.stop(coast);
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
void defensiveZoneAuton()
{
	gyro1.resetRotation();
	//new
	robot=point(0,0);
	Stanley::setPath(std::vector<point>{
		{0,0},
		{0,20},
		{10,20},
		{12,48}
	});
	wingL.set(true);
	drive(100,100,100);
	intake.spin(fwd,12,volt);
	wingL.set(false);
	Stanley::run(fast);
	wait(250,msec);
	inchDrive(-20);
	rotateTo(90);
	drive(50,50,0);
	waitUntil(gyro1.pitch()>15);
	drive_brake();
	intake.spin(reverse,12,volt);
	inchDrive(-24);
	return;
	wingR.set(true);
	wait(500, msec);
	drive(-100, 100, 250);
	rotateTo(-30);
	wingR.set(false);
	intake.spin(reverse, 12, volt);
	wait(1, sec);
	inchDrive(15);
	rotateTo(160);
	inchDrive(-20, 2, 1);
	inchDrive(5);
	rotateTo(135);
	inchDrive(30);
	wait(500, msec);
	rotateTo(90);
	wait(500, msec);
	inchDrive(23.5, 1, 2);
	drive_brake();
	toggleBlocker();
}

void skillsAuton()
{
	gyro1.resetRotation();
	robot = point(0, 0);
	waitUntil(!gyro1.isCalibrating());
	robot = point(0, 0);
	inchDrive(20);
	rotateTo(45);
	inchDrive(-17, 1, 3);
	// rotateTo(60);
	Controller1.rumble("-.-.-");
	drive(-10, -10, 0);
	cata.spin(fwd, 12 * .8, volt);
	wait(.35, sec);
	drive_brake(hold);
	rotateTo(100);
	// drive(-0.1,-0.1,0);
	//wait(40, sec);//!wait 
	cata.stop(coast);
	inchDrive(30);
	wait(0.45, sec);
	rotateTo(90);
	inchDrive(60);
	rotateTo(0);
	inchDrive(30);
	rotateTo(-90);
	inchDrive(20);
	rotateTo(0);
	inchDrive(50);
	rotateTo(90);
	wingL.set(true);
	wingR.set(true);
	inchDrive(30);

	/*
	Stanley::setPath(std::vector<point>{
		{robot},
		{robot.x + 20, robot.y},
		{75.857, 0.857},
		{90, 15},
		{90, 15},
		{104.14, 29.14},
		{75, 40},
		{75, 63}});
	std::cout << "path succsesful";
	Stanley::run(slow);
	rotateTo(90);
	flapWings();
	inchDrive(30, 1.25, 3);
	flapWings();
	inchDrive(-12);
	/*Stanley::setPath(std::vector<point>{
		{robot},
		{robot.x + 5, robot.y + 3}}); */
}
