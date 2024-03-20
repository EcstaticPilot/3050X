#include "vex.h"
#include "stanley.h"
#include <vector>
using namespace vex;
void flapWings();
void toggleBlocker();

void testAuton()
{
	wingRD.set(true);
	inchDrive(-12);
	wingRD.set(false);
	inchDrive(24);
	rotateTo(10);
	intake.spin(fwd, -12, volt);
	wait(250, msec);
	rotateTo(180);
	inchDrive(-15, 2, 1);
	inchDrive(5);
	rotateTo(-135);
	inchDrive(24);
	rotateTo(-90);
	inchDrive(40);

	rotateTo(9.94638230542056);
	inchDrive(140.42130298026836);
	rotateTo(56.11959576485096);
	inchDrive(172.19346934375832);
	Stanley::setPath(std::vector<point>{
		{29.36, 26.922},
		{57.803, 46.021},
		{1.524, 68.574},
		{-28.547, 47.646}});
	Stanley::run(meduim);
}

void offensiveZoneAuton()
{
	lift.spin(fwd, -4, volt);
	std::cout << "auton start";
	//! arc drive now works differently maybe fixed needs testing
	gyro1.resetRotation();
	Stanley::setPath(std::vector<point>{{0, 0}, {0, 20}, {-5, 20}, {-10, 40}});
	Stanley::run();
	// arcDrive_length(30, -37.5);
	rotateTo(60, 0.5);
	intake.spin(fwd, -12, volt);
	wait(500, msec);
	intake.stop(coast);
	//-30,65

	// float angle = atan((-30-robot.x)/(65-robot.y));
	rotateTo(310);
	// turnToObject(TRIBALL,-50,2,15,false);
	// wait(100, msec);
	lift.spin(fwd, -2, volt);
	intake.spin(fwd, 100, pct);
	inchDrive(24, 1.5, 1);
	inchDrive(-2);

	rotateTo(90);
	flapWings();
	// wait(500, msec);
	intake.spin(reverse, 100, pct);
	inchDrive(31, 1.25, .75);
	intake.stop(coast);
	flapWings();
	inchDrive(-10);
	rotateTo(-130);
	turnToObject(TRIBALL, -138);
	intake.spin(forward, 100, pct);
	inchDrive(30);
	wait(100, msec);
	inchDrive(-19);
	rotateTo(70);
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

void offensiveZoneAuton2()
{
	robot = point(34.84582346787071,-53.74180937173063); 
	intake.spin(fwd, 12,volt);
	Stanley::setPath(std::vector<point>{ 
		{ 34.846,-53.742 }, 
		{ 34.643,-27.938 }, 
		{ 22.655,-27.328 }, 
		{ 23.468,-4 }}); 
	Stanley::run(fast); 
	wait(100, msec);
	rotateTo(90); 
	intake.spin(fwd, -12,volt);
	
	
	inchDrive(21); 
	rotateTo(-90); 
	//wait(100, msec);
	
	inchDrive(18); 
	rotateTo(90); 
	//wait(100, msec);

	inchDrive(35.075199756719336);      
	rotateTo(-117.85956073062702); 
	turnToObject(TRIBALL);
	//wait(100, msec);

	inchDrive(43.26218692366509); 
	rotateTo(71.65052842875856); 
	//wait(100, msec);

	inchDrive(35.241639459133054);
}
void defensiveZoneAuton()
{
	// code for defensive zone auton
	inchDrive(4);
	wingLD.set(true);
	wait(100, msec);
	drive(-50, -25, 500);
	wingLD.set(false);
	rotateTo(-45);
	inchDrive(17.6);

	rotateTo(-10);
	intake.spin(fwd, -12, volt);
	wait(250, msec);
	rotateTo(170);
	inchDrive(-15, 2, 1);
	inchDrive(10);
	rotateTo(135);
	inchDrive(27, 0.75);
	rotateTo(90);
	wait(500, msec);
	inchDrive(30.69); // funny number
	drive_brake(brake);
	std::cout << robot.x << std::endl;
}

void skillsAuton()
{

	gyro1.resetRotation();
	robot = point(0, 0);
	waitUntil(!gyro1.isCalibrating());
	robot = point(0, 0);
	inchDrive(25);
	rotateTo(80);
	inchDrive(-25, 1, 1);
	// rotateTo(60);
	lift.spin(fwd, 12, volt);
	rotateTo(70);
	wingRD.set(true);

	Controller1.rumble("-.-.-");
	cata.spin(fwd, 12 * .95, volt);
	wait(.35, sec);
	drive_brake(hold);
	drive(-0.1, -0.1, 0);
	float waitTime = 0; // 34; // 32
	// driver control inside auton lol
	timer timer1 = timer();
	// driver control inside auton lol
	while (1)
	{
		float pos = cata.position(degrees);
		cata.setPosition(fmod(pos, 720), degrees);
		std::cout << pos << std::endl;
		if ((cata.power() > 0.000005 && cata.power() < 1) && cata.velocity(pct) > 20)
		{
			cata.resetPosition();
			std::cout << "resetPosition" << std::endl;
		}
		if (timer1.time() / 1000.0 < waitTime || cata.position(degrees) < 550)
		{
			cata.spin(fwd, 12 * 0.95, volt);
		}
		else
		{
			cata.stop(hold);
			if (timer1.time() > waitTime * 1000)
			{
				break;
			}
		}
	}
	// waitUntil(cata.position(degrees)>570);
	cata.stop(hold);

	// cata reset sequence;
	cata.resetPosition();
	cata.stop(coast);
	wait(1.25, sec);
	cata.resetPosition();
	cata.spin(fwd, 10, volt);
	waitUntil(cata.position(degrees) > 600);
	cata.stop(hold);
	// end of cata reset sequence

	wingRD.set(false);
	Controller1.rumble("...");
	lift.spin(fwd, -12, volt);
	rotateTo(90);
	Stanley::setPath(std::vector<point>{{robot.x, robot.y}, {robot.x + 2, robot.y - 7}, {-10, 5}, {75, 8}});
	Stanley::run(fast);
	rotateTo(-110);
	lift.stop(coast);
	wingLD.set(true);
	wingRD.set(true);
	float angle = gyro1.rotation(degrees) + 180;
	float mag = 0;
	point p2(robot.x + mag * sin(angle), robot.y + mag * cos(angle));
	Stanley::setPath(std::vector<point>{{robot.x, robot.y}, {p2}, {90, 14}, {92.9, 21.8}});
	Stanley::run(meduim, true);
	wingLD.set(false);
	wingRD.set(false);
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
	wingLD.set(true);
	wingRD.set(true);
	inchDrive(-33, 2, 1);
	inchDrive(30);
	rotateTo(-110);
	inchDrive(-40, 2, 1);
}
