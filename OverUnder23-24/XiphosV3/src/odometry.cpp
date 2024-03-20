#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
/*
 ██████╗   ██████╗    ██████╗   ███╗   ███╗  ███████╗  ████████╗  ███████╗  ██████╗   ██╗   ██╗
██╔═══██╗  ██╔══██╗  ██╔═══██╗  ████╗ ████║  ██╔════╝  ╚══██╔══╝  ██╔════╝  ██╔══██╗  ╚██╗ ██╔╝
██║   ██║  ██║  ██║  ██║   ██║  ██╔████╔██║  █████╗       ██║     █████╗    ██████╔╝   ╚████╔╝
██║   ██║  ██║  ██║  ██║   ██║  ██║╚██╔╝██║  ██╔══╝       ██║     ██╔══╝    ██╔══██╗    ╚██╔╝
╚██████╔╝  ██████╔╝  ╚██████╔╝  ██║ ╚═╝ ██║  ███████╗     ██║     ███████╗  ██║  ██║     ██║
 ╚═════╝   ╚═════╝    ╚═════╝   ╚═╝     ╚═╝  ╚══════╝     ╚═╝     ╚══════╝  ╚═╝  ╚═╝     ╚═╝

*/

point robot = point(0, 0);

#include <string>

void point::print()
{
	std::cout << x << "," << y << std::endl;
}
float round(float var)
{
	float value = (int)(var * 100 + .5);
	return (float)value / 100;
}
bool point::equals(point p1)
{
	if ((round(this->x) == round(p1.x)) and (round(this->y) == round(p1.y)))
	{
		return true;
	}
	return false;
}
/**
 * @brief odometry
 */
int odometery()
{
	RF.resetPosition();
	LF.resetPosition();
	gyro1.resetHeading();

	double prevHeading = gyro1.rotation();

	double deltaHeading = 0; // change in heading
	double Heading = M_PI;
	double distFromCenterL = 6;
	double distFromCenterB = 5;
	double lRad = 1.625;
	double rRad = 1.625; // radius of tracking wheel
	double bRad = 1.625; // radius of tracking wheel

	// radius of tracking wheel
	double lEncoder = 0; // declaring encoder variable left
	double rEncoder = 0; // declaring encoder variable right
	double bEncoder = 0; // declaring encoder variable right

	double distL = 0; // distance left encoder has traveled
	double distR = 0; // distance right encoder has traveled
	double distB = 0;

	double prevLE = lEncoder; // create previous encoder value left
	double prevRE = rEncoder; // create previous encoder value right
	double prevBE = bEncoder; // create previous encoder value right

	double averageHeading; //
	double deltaX;
	double deltaY;
	// gyro1.datarate(10);

	waitUntil(!gyro1.isCalibrating());
	robot = point(0, 0);
	RF.resetPosition();
	LF.resetPosition();
	RF.setPosition(0, degrees);
	LF.setPosition(0, degrees);
	while (true)
	{
		lEncoder = LF.position(degrees) * 3 / 4;
		rEncoder = RF.position(degrees) * 3 / 4;
		// bEncoder = RotationB.position(degrees);
		//  convert encoder distance into distance traveled
		distL = ((lEncoder - prevLE) * M_PI / 180) * lRad;
		distR = ((rEncoder - prevRE) * M_PI / 180) * rRad;
		distB = ((bEncoder - prevBE) * M_PI / 180) * bRad;
		// convert encoder distance into disntance traveled

		prevLE = lEncoder; // create previous encoder value left
		prevRE = rEncoder; // create previous encoder value right
		prevBE = bEncoder;

		Heading = ((gyro1.rotation() * M_PI / 180));
		Heading = fmod(Heading, 2 * M_PI);
		deltaHeading = Heading - prevHeading; // calculate change in heading

		prevHeading = Heading;

		if (deltaHeading == 0 || true)
		{
			deltaX = distB;
			if (RF.installed() && LF.installed())
				deltaY = (distL + distR) / 2;
		}
		else
		{
			deltaX = 2 * sin(deltaHeading / 2) * ((distB / deltaHeading) + distFromCenterB);
			deltaY = 2 * sin(deltaHeading / 2) * ((distR / deltaHeading) - distFromCenterL);
		}

		averageHeading = prevHeading + (deltaHeading / 2);

		robot.x += (deltaY * sin(averageHeading)) + (deltaX * cos(averageHeading));
		robot.y += (deltaY * cos(averageHeading)) - (deltaX * sin(averageHeading));

		if (deltaX not_eq 0 || deltaY not_eq 0)
		{
			//    std::cout <<robot.x << "," <<robot.y << std::endl;
		}

		this_thread::sleep_for(10);
	}
	return 1;
}
