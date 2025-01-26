#include "vex.h"
#include <vector>
#include "pid.h"

using namespace vex;

float C = M_PI * 3.25;
int signOf(float x);

void PID::update()
{
	if (signOf(prevError) == signOf(error))
	{
		totalError += error;
	}
	else
	{
		totalError = 0;
	}
	output = kp * error + (ki * totalError) + (kd * (prevError - error));
	prevError = error;
}

void PID::print(float error)
{
	std::cout << error << "," << kp * error << "," << ki * totalError << "," << (kd * (prevError - error)) << "," << kp * error + (ki * totalError) + (kd * (prevError - error)) << std::endl;
}
double driveCurve(double x)
{
	return pow(x, 2) / 100 * (x / fabs(x));
	// return pow(M_E,(fabs(x)-100)/100)*x;
}
/**
 * @brief drives the robot using speed
 *
 * @param lSpeed
 * @param rSpeed
 * @param wt
 */
void drive(double lSpeed, double rSpeed, double wt)
{
	LF.spin(forward, lSpeed, pct);
	LM.spin(forward, lSpeed, pct);
	LB.spin(forward, lSpeed, pct);

	RF.spin(forward, rSpeed, pct);
	RB.spin(forward, rSpeed, pct);
	RM.spin(forward, rSpeed, pct);

	wait(wt, msec);
}
/**
 * @brief drives the robot based on voltage
 * @param lSpeed left side speed in percent
 * @param rSpeed right side speed in percent
 */
void voltDrive(double lSpeed, double rSpeed, double wt)
{
	LF.spin(forward, lSpeed * 120, voltageUnits::mV);
	LM.spin(forward, lSpeed * 120, voltageUnits::mV);
	LB.spin(forward, lSpeed * 120, voltageUnits::mV);

	RF.spin(forward, rSpeed * 120, voltageUnits::mV);
	RM.spin(forward, rSpeed * 120, voltageUnits::mV);
	RB.spin(forward, rSpeed * 120, voltageUnits::mV);
	wait(wt, msec);
}
/**
 * @brief drives the robot based on voltage and feedforward kp
 *
 * @param lSpeed
 * @param rSpeed
 * @param wt
 */
void ffDrive(double lSpeed, double rSpeed, double wt)
{
	float kp = 0.1;
	float lfSpeed = lSpeed + (kp * (lSpeed - LF.velocity(pct)));
	float rfSpeed = rSpeed + (kp * (rSpeed - RF.velocity(pct)));

	float lbSpeed = lSpeed + (kp * (lSpeed - LB.velocity(pct)));
	float rbSpeed = rSpeed + (kp * (rSpeed - RB.velocity(pct)));

	float rmSpeed = rSpeed + (kp * (rSpeed - RM.velocity(pct)));
	float lmSpeed = rSpeed + (kp * (rSpeed - LM.velocity(pct)));

	LF.spin(forward, lfSpeed * 120, voltageUnits::mV);
	RF.spin(forward, rfSpeed * 120, voltageUnits::mV);

	LB.spin(forward, lbSpeed * 120, voltageUnits::mV);
	RB.spin(forward, rbSpeed * 120, voltageUnits::mV);

	LM.spin(forward, lmSpeed * 120, voltageUnits::mV);
	RM.spin(forward, rmSpeed * 120, voltageUnits::mV);
	wait(wt, msec);
}

/**
 * @brief brakes the robot based on brake type
 * @param Brake brake type
 */
void drive_brake(vex::brakeType Brake)
{
	LF.stop(Brake);
	RF.stop(Brake);
	LB.stop(Brake);
	LM.stop(Brake);
	RM.stop(Brake);
	RB.stop(Brake);
}

/**
 * @brief drives the robot a certain distance using PID
 * @param dist distance in inches
 * @param speedMod speed modifier
 * @param accuracy accuracy in inches
 */
void inchDrive(double target, double speedMod, double timeout, double accuracy)
{
	LB.resetPosition();
	Brain.Timer.clear();
	// std::cout<<accuracy;
	float kp = 6, ki = 0.01, kd = 3.44;
	// HAVE_INITFINI_ARRA;:)

	float speed = 100;

	PID pid = PID(kp, ki, kd, target, 30);

	while (fabs(pid.error) > accuracy)
	{
		pid.error = (target) - ((LB.position(turns) * 2 * M_PI) * 1.625 * 3 / 4);
		pid.update();
		speed = pid.output;
		//std::cout << "motor effecieny = " << LB.efficiency(percent) << "," << RB.efficiency(percent) << std::endl;
		//pid.print(pid.error);
		if ((Brain.Timer.value() > timeout))
		{
			break;
		}
		voltDrive(speed * speedMod, speed * speedMod, 10);
	}
	drive_brake(brake);
}

/**
 * @brief drives the robot with an arc to a final angle
 *
 * @param finalAngle the final angle to turn to
 * @param radius the radius of the arc, positive is to the right, negative is to the left
 * @param isGlobal if the final angle of the arc is global
 * @param isReverse whether or not to follow the arc backwards
 * @param accuracy accuracy of the distance
 */
void arcDrive(float finalAngle, float radius, bool isGlobal, bool isReverse, float accuracy)
{
	if (isGlobal)
	{
		finalAngle -= gyro1.rotation(degrees);
	}
	float dist = fabs(finalAngle) * fabs(radius) * M_PI / 180;

	if (isReverse)
	{
		dist *= -1;
	}
	float curvature = 1 / radius;
	LB.resetPosition();
	RB.resetPosition();

	float distRC = radius - TrackWidth / 2;
	float distLC = radius + TrackWidth / 2;
	float speedMod = distLC / distRC;
	std::cout << speedMod << std::endl;
	float kp = 5.75, ki = 0, kd = 7;

	float vL;
	float vR;
	float normFactor;

	float avgPos;

	float speed;
	float maxSpeed = 100;

	PID pid = PID(kp, ki, kd, dist, 30);

	while (fabs(pid.error) > accuracy)
	{
		avgPos = ((LB.position(turns) + RB.position(turns)) / 2) * 3 / 4; // average position of the left and right encoders w/ gear ratio
		pid.error = dist - (avgPos * M_TWOPI * 1.625);					  // convert to inches using radians and radius
		pid.update();
		speed = pid.output;
		// pid.print();

		vL = speed * (2 + (curvature * TrackWidth)) / 2;
		vR = speed * (2 - (curvature * TrackWidth)) / 2;
		normFactor = maxSpeed / fmax(fabs(vL), fabs(vR));

		// clip speed if necessary
		if (normFactor < 1)
		{
			vL *= normFactor;
			vR *= normFactor;
		}

		// std::cout << pid.error << "," << gyro1.yaw(degrees) << std::endl;
		//! the *speedMod makes it work but its not mathemtically right
		std::cout << vL << "," << vR << std::endl;
		drive(vL, vR, 10);
	}
	std::cout << "done" << std::endl;
	drive_brake(brake);
}

/**
 * @brief drives the robot with an arc to a final angle
 *
 * @param finalAngle the final angle to turn to
 * @param radius the radius of the arc, positive is to the right, negative is to the left
 * @param isGlobal if the final angle of the arc is global
 * @param isReverse whether or not to follow the arc backwards
 * @param accuracy accuracy of the distance
 */
void arcDrive_length(float dist, float radius, bool isGlobal, bool isReverse, float accuracy)
{
	float curvature = 1 / radius;
	LB.resetPosition();
	RB.resetPosition();

	float distRC = radius - TrackWidth / 2;
	float distLC = radius + TrackWidth / 2;
	float speedMod = distLC / distRC;
	std::cout << speedMod << std::endl;
	float kp = 5.75, ki = 0, kd = 7;

	float vL;
	float vR;
	float normFactor;

	float avgPos;

	float speed;
	float maxSpeed = 100;

	PID pid = PID(kp, ki, kd, dist, 30);

	while (fabs(pid.error) > accuracy)
	{
		avgPos = ((LB.position(turns) + RB.position(turns)) / 2) * 3 / 4; // average position of the left and right encoders w/ gear ratio
		pid.error = dist - (avgPos * M_TWOPI * 1.625);					  // convert to inches using radians and radius
		pid.update();
		speed = pid.output;
		// pid.print();

		vL = speed * (2 + (curvature * TrackWidth)) / 2;
		vR = speed * (2 - (curvature * TrackWidth)) / 2;
		normFactor = maxSpeed / fmax(fabs(vL), fabs(vR));

		// clip speed if necessary
		if (normFactor < 1)
		{
			vL *= normFactor;
			vR *= normFactor;
		}

		// std::cout << pid.error << "," << gyro1.yaw(degrees) << std::endl;
		//! the *speedMod makes it work but its not mathemtically right
		std::cout << vL << "," << vR << std::endl;
		drive(vL, vR, 10);
	}
	std::cout << "done" << std::endl;
	drive_brake(brake);
}
/**
 * @brief rotates the robot to an angle gloablly
 *
 * @param dir direction to turn to
 * @param accuracy accuracy of the turn
 */
void rotateTo(double dir, double accuracy)
{
	float kp = 0.4, ki = 0.00127, kd = 0.9;
	float speed = 0;

	PID pid = PID(kp, ki, kd, dir - gyro1.rotation(degrees), 20);

	while (true)
	{
		pid.error = dir - gyro1.rotation(degrees);

		if (fabs(pid.error) > 180)
		{
			pid.error = pid.error - (signOf(pid.error) * 360);
		}
		// pid.print(pid.error);
		pid.update();
		speed = pid.output;

		// std::cout << pid.error << "," << pid.output << std::endl;
		if (fabs(pid.error) < accuracy && fabs(LM.velocity(pct)) < 10)
		{
			break;
		}
		drive(speed, -speed, 10);
	}
	//std::cout << "DONE" << std::endl;
	drive_brake(brake);
	// wait(1, sec);
	//std::cout << pid.error << std::endl;
}

void rotateToPoint(point target){
	double angle = atan2(target.y-robot.y,target.x-robot.y);
	rotateTo(angle);
}
const float radius = 3.25 / 2;
double RPMtoIPS(double rpm)
{
	// inches per second
	float ips = rpm * 2 * 3.1415926535897932384626433832795 * radius / 60;
	// take into account gear ratio
	return ips * 3 / 4;
}
double IPStoRPM(double ips)
{
	float rpm = ips * 60 / (2 * 3.1415926535897932384626433832795 * radius);
	return rpm * 4 / 3;
}
//!				CAMERA 			STUFF

float distFromClosetObject()
{
	float output = 0;
	float dist = fabs(camera.objects[0].centerX - 157.5);
	;
	for (int i = 0; i < camera.objectCount; i++)
	{
		if (!camera.objects[i].exists)
		{
			break;
		}
		dist = camera.objects[i].centerX - 157.5;
		if (fabs(dist) < fabs(output))
		{
			output = dist;
		}
	}
	return output;
}

float average(std::vector<float> vector);

/**
 * @brief turn to an object
 *
 * @param target target signaute
 * @param angle estimated angle
 * @param tolerance how far away from expected angle to look for object
 * @param bigestObject whether or not to find the bigest objet or closest object
 */
void turnToObject(vision::signature target, float angle, float accuracy, float tolerance, bool bigestObject)
{
	float kp = .35, ki = 0.0015, kd = 1;
	float speed = 0;
	PID pid = PID(kp, ki, kd, angle - gyro1.rotation(degrees), 20);
	std::vector<float> SMA = std::vector<float>(6);
	while (true)
	{
		pid.error = angle - gyro1.rotation(degrees);
		camera.takeSnapshot(target);
		wait(5, msec);
		// use camera
		if (pid.error < tolerance && camera.largestObject.exists)
		{
			if (!bigestObject)
			{
				SMA.push_back(-1 * (distFromClosetObject()));
			}
			else
			{
				SMA.push_back(-1 * (camera.largestObject.centerX - 157.5));
			}
			while (SMA.size() > 6)
			{
				SMA.erase(SMA.begin());
			}
			pid.error = -1 * (camera.largestObject.centerX - 157.5);
			std::cout << "camera: " << pid.error << std::endl;
			pid = PID(0.1, 0, 0, pid.error, 20);
		}
		else
		{
			pid = PID(kp, ki, kd, pid.error, 20);
		}
		if (fabs(pid.error) < accuracy && fabs(LM.velocity(pct)) < 5)
		{
			break;
		}
		pid.update();
		speed = pid.output;
		pid.print(pid.error);
		// std::cout << pid.error << "," << pid.output << std::endl;
		drive(speed, -speed, 5);
	}
	std::cout << pid.error << std::endl;
	drive_brake(brake);
}

void DriveToObject(vision::signature target, float dist, float accuracy)
{
	PID drivePID = PID(5.25, 0.025, 8.25, dist);
	PID cameraPID = PID(1, 0, 0, 0);
	std::vector<float> SMA = std::vector<float>(6);
	LB.resetPosition();
	RB.resetPosition();
	while (fabs(drivePID.error) > accuracy)
	{
		drivePID.error = (dist) - ((LB.position(turns) + RB.position(turns) * M_PI) * 1.625 * 3 / 4);
		camera.takeSnapshot(target);
		wait(5, msec);
		cameraPID.error = -1 * distFromClosetObject();

		if (camera.largestObject.width > 100)
		{
			cameraPID.error = 0;
		}
		std::cout << camera.largestObject.width << std::endl;
		drivePID.update();
		cameraPID.update();
		// cameraPID.print(cameraPID.error);
		float left = fmax(drivePID.output, 100) + cameraPID.output;
		float right = fmax(drivePID.output, 100) - cameraPID.output;

		std::cout << "left" << left << "right" << right << std::endl;
		drive(left, right, 10);
	}
	drive_brake();
}

//!
//!
//!                EXPERIMENTAL UNTESTED STUFF AFTER THIS POINT
//!
//!
void DriveToPoint(double targetX, double targetY, float speedMult = 1)
{
	// T=turn
	// D=distance
	float tError = gyro1.rotation(degrees) - atan2(targetY - robot.y, robot.x - targetX);
	float Tkp = 1, Tki = 0, Tkd = 1;
	float tErrorOld = tError;
	float tSum = 0;

	float dError =
		sqrt((targetX - robot.x) * (targetX - robot.x) + (targetY - robot.y) * (targetY - robot.y));
	float Dkp = 0, Dki = 0, Dkd = 0;
	float dErrorOld = dError;
	float dSum = 0;

	float accuracy = 1;

	while ((fabs(targetX - robot.x) > accuracy) || (fabs(targetY - robot.y) > accuracy))
	{

		tError = -gyro1.orientation(yaw, degrees) -
				 (atan2(targetY - robot.y, robot.x - targetX) * 180 / M_PI);
		dError =
			sqrt((targetX - robot.x) * (targetX - robot.x) + (targetY - robot.y) * (targetY - robot.y));
		float rSpeed = (Dkp * dError) + (Dki * dSum) +
					   (Dkd * (dError - tErrorOld)) + (Tkp * tError) +
					   (Tki * tSum) + (Tkd * (tError - tErrorOld));

		float lSpeed =
			(Dkp * dError) + (Dki * dSum) + (Dkd * (dError - dErrorOld)) -
			((Tkp * tError) + (Tki * tSum) + (Tkd * (tError - tErrorOld)));
		if (speedMult < 0)
		{
			double idk;
			idk = rSpeed;
			rSpeed = lSpeed;
			lSpeed = idk;
		}
		std::cout << lSpeed << "," << rSpeed << "," << dError << "," << tError << std::endl;
		drive((lSpeed)*speedMult, (rSpeed)*speedMult, 12);
		dErrorOld = dError;
		dSum += dError;

		tErrorOld = tError;
		tSum += tError;
	}
	drive_brake(brake);
}
//! no workie
//?te
void DriveToPoint2(float targetX, float targetY)
{

	rotateTo((atan2(targetY - robot.y, robot.x - targetX) * 180 / M_PI));
	std::cout << gyro1.rotation() << std::endl;
	wait(1, sec);
	inchDrive(hypot(targetY - robot.y, targetX - robot.x));
	std::cout << robot.x << "," << robot.y << std::endl;
}
/**
 * @brief Ramsete control algorithm
 */
void RAMSETE(float targetX, float targetY, float targetAngle,
			 float accuracy = 1)
{
	// https://wiki.purduesigbots.com/software/control-algorithms/ramsete
	// θ - copy and paste theta
	float errorX = targetX - robot.x;
	float errorY = targetY - robot.y;
	float smallScalar = 0.01;
	float beta = 0.3;
	float zeta = .2;
	while (!((fabs(errorX) < accuracy) && (fabs(errorY)) < accuracy))
	{
		errorX = targetX - robot.x;
		errorY = targetY - robot.y;
		float errorθ = -gyro1.rotation() - (atan2(targetY - robot.y, robot.x - targetX) * 180 / M_PI);
		float ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
		float ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
		float eθ = errorθ;

		float vd = errorY * smallScalar;
		float wd = eθ * smallScalar;
		float k = 2 * zeta * sqrt(pow(wd, 2) + beta * pow(vd, 2));
		float v = (vd * cos(eθ)) + (k * ey);
		float w = wd + k * eθ + (beta * vd * sin(eθ) * ex) / eθ;

		float linearMotorVelocity = v / (M_PI * 3.25);
		float left = linearMotorVelocity;  //+ w;
		float right = linearMotorVelocity; // - w;
		std::cout << v << "," << w << "," << eθ << std::endl;
		drive(left, right, 10);
	}
	drive_brake(brake);
	Controller1.rumble(".");
}

/**
 * @brief Drives to a point using the gyro
 * // hvchfchf gnd
 */
//! no workie
void DriveToPoint3(float targetX, float targetY, float targetAngle,
				   float accuracy = 1)
{
	// θ - copy and paste theta
	float errorX = targetX - robot.x;
	float errorY = targetY - robot.y;
	float
		vp = 15,
		vi = 0,
		vd = 3;
	float
		wp = .5,
		wi = 0.0,
		wd = 0;
	float ySum = 0;
	float xSum = 0;
	float ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
	float ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
	while (!((fabs(errorX) < accuracy) && (fabs(errorY)) < accuracy))
	{
		errorX = targetX - robot.x;
		errorY = targetY - robot.y;

		float prevX = ex;
		float prevY = ey;

		ey = cos(gyro1.rotation()) * errorY + sin(gyro1.rotation()) * errorX;
		ex = -sin(gyro1.rotation()) * errorY + cos(gyro1.rotation()) * errorX;
		std::cout << ex << "," << ey << std::endl;
		float v = vp * ey + vi * ySum + vd * (prevY - ey);
		float w = wp * ex + wi * xSum + wd * (prevX - ex);

		float left = v + w;
		float right = v - w;

		ySum += ey;
		xSum += ex;
		drive(left, right, 10);
	}
	drive_brake(brake);
}

void driveAtAngle(float angle, float baseSpeed, float time)
{
	time = time * 1000;
	float kp = 6;
	float ki = 0.00;
	float kd = 30;
	float error = angle - gyro1.rotation();
	float prevError = error;
	float sum = 0;
	float speed;
	float vL, vR;
	float normFactor;
	float maxSpeed = 100;
	Brain.Timer.reset();
	Brain.Timer.clear();
	while (Brain.Timer.time() <= time)
	{
		error = angle - gyro1.rotation();
		speed = (kp * error) + (ki * sum) + (kd * (error - prevError));
		vL = baseSpeed + speed;
		vR = baseSpeed - speed;

		// calculate the normalization factor
		normFactor = maxSpeed / fmax(fabs(vL), fabs(vR));

		// clip speed if necessary
		if (normFactor < 1)
		{
			vL *= normFactor;
			vR *= normFactor;
		}

		// drive using voltDrive
		voltDrive(vL, vR, 10);

		if (fabs(error) / error != fabs(prevError) / prevError)
		{
			sum = 0;
		}
		else
		{
			sum += error;
		}
		std::cout << Brain.Timer.time() << "," << error << "," << speed << "," << kp * error << "," << ki * sum << "," << kd * (error - prevError) << std::endl; // pid tuning config
		wait(10, msec);
		prevError = error;
	}
	drive_brake(coast);
}