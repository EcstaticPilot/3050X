#pragma once

//
double driveCurve(double x);
void drive(int lSpeed, int rSpeed, double wt);
void voltDrive(double lSpeed, double rSpeed, double wt);
void ffDrive(double lSpeed, double rSpeed, double wt);
void drive_brake(vex::brakeType Brake = brake);
void inchDrive(double target, double speedMod);
void rotate(double dir, double accuracy=1);
void stanley(float points[][2], int length, bool isReversed = false);
void curveDrive(double x, double y, double speed, double wt);
void purePursuit(float points[][2], int length);

