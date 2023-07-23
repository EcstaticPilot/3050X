#pragma once

//
void drive(int lSpeed, int rSpeed, double wt);
void voltDrive(double lSpeed, double rSpeed, double wt);
void ffDrive(double lSpeed, double rSpeed, double wt);
void drive_brake(vex::brakeType Brake = brake);
void forward_dist(float dist);
void inchDrive(double target, double speedMod);
void rotate(double dir, double accuracy=1);
void DriveToPoint(double targetX, double targetY, float speedMult);
void DriveToPoint2(float targetX, float targetY);
void RAMSETE(float targetX, float targetY, float targetAngle,
             float accuracy);
void DriveToPoint3(float targetX, float targetY, float targetAngle,
                   float accuracy);
void driveAtAngle(float angle, float baseSpeed, float time);
void stanley(float points[][2], int length);
void curveDrive(double x, double y, double speed, double wt);
void purePursuit(float points[][2], int length);

