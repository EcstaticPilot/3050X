#include "stdio.h"
#include "vex.h"
#include <math.h>
#include <iostream>
#include <algorithm>
#include <monteCarlo.h>
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
#include <vector>
void point::print()
{
	std::cout << x << "," << y << std::endl;
}
float round(float var)
{
	float value = (int)(var * 100 + .5);
	return (float)value / 100;
}
float point::angleTo(point p1){
	return atan2(p1.x - this->x, p1.y - this->y) * 180/M_PI;
}
bool point::equals(point p1)
{
	if ((round(this->x) == round(p1.x)) and (round(this->y) == round(p1.y)))
	{
		return true;
	}
	return false;
}
point point::operator+(point p1)
{
	return point(this->x + p1.x, this->y + p1.y);
}
point point::operator+=(point p1)
{
	this->x += p1.x;
	this->y += p1.y;
	return *this;
}
double deltaHeading = 0; // change in heading
	double Heading = M_PI;
	double distFromCenterL = 6;
	double distFromCenterB = 1;
	double lRad = 1.625;
	double rRad = 1.625; // radius of tracking wheel
	double bRad = 1.375; // radius of tracking wheel
/**
 * @brief odometry
 */
int odometery()
{
	RF.resetPosition();
	LF.resetPosition();
	gyro1.resetHeading();

	double prevHeading = gyro1.rotation();

	

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
	int i =0;
	waitUntil(!gyro1.isCalibrating());
	robot = point(0, 0);
	RF.resetPosition();
	LF.resetPosition();
	RF.setPosition(0, degrees);
	LF.setPosition(0, degrees);
	float angle = gyro1.rotation();
	if(GPS.installed()){
			gyro1.setRotation(GPS.rotation(),degrees);
			float x = GPS.xPosition(inches);
			float y = GPS.yPosition(inches);
			robot = point(x,y);
	}
	while (true)
	{
		if(GPS.installed() and GPS.quality() ==100 and i == 1){
			angle = (GPS.rotation() + gyro1.rotation())/2;
			float x = GPS.xPosition(inches);
			float y = GPS.yPosition(inches);
			GPS.setRotation(angle, degrees);
			gyro1.setRotation(angle,degrees);
			robot.x = robot.x + (x-robot.x)*0.25;
			robot.y = robot.y + (y-robot.y)*0.25;
			i=0;
		}else{
			angle = gyro1.rotation();
		}
		i++;
		lEncoder = LF.position(degrees) * 3 / 4;
		rEncoder = RF.position(degrees) * 3 / 4;
		bEncoder = rotationB.position(degrees);
		//  convert encoder distance into distance traveled
		distL = ((lEncoder - prevLE) * M_PI / 180) * lRad;
		distR = ((rEncoder - prevRE) * M_PI / 180) * rRad;
		distB = ((bEncoder - prevBE) * M_PI / 180) * bRad;
		// convert encoder distance into disntance traveled

		prevLE = lEncoder; // create previous encoder value left
		prevRE = rEncoder; // create previous encoder value right
		prevBE = bEncoder;

		Heading = ((angle * M_PI / 180));
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
			std::cout <<robot.x << "," <<robot.y << std::endl;
		}

		this_thread::sleep_for(10);
	}
	return 1;
}

pose odomStep(float dx, float dy,float heading, float deltaHeading){
	float lx = 2 * sin(deltaHeading / 2) * ((dx / deltaHeading) + distFromCenterB);
	float ly = 2 * sin(deltaHeading / 2) * ((dy / deltaHeading) - distFromCenterL);
	float gx = (ly * sin(heading)) + (lx * cos(heading));
	float gy = (ly * cos(heading)) - (lx * sin(heading));
	return pose(gx,gy,heading);
}