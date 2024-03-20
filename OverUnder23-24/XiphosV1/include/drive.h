#pragma once
enum stanleyConfig{
    slow,
    meduim,
    fast
};
//trackwidth to be used globally
inline const float TrackWidth =12;
double driveCurve(double x);
void drive(double lSpeed, double rSpeed, double wt);
void voltDrive(double lSpeed, double rSpeed, double wt = 0);
void ffDrive(double lSpeed, double rSpeed, double wt);
void drive_brake(vex::brakeType Brake = brake);
void inchDrive(double target, double speedMod = 1,double timeout = 1000, double accuracy = 1);
void rotateTo(double dir, double accuracy=1);
void arcDrive(float finalAngle, float radius, bool isGlobal = true, bool isReverse = false, float accuracy = 1);
void arcDrive_length(float dist, float radius, bool isGlobal=true, bool isReverse=false, float accuracy=1);
float distFromClosetObject();
void turnToObject(vision::signature target, float angle = 0, float accuracy =2, float tolerance = 20, bool bigestObject = false);
void DriveToObject(vision::signature target, float dist, float accuracy=0.5);
void curveDrive(double x, double y, double speed, double wt);
void purePursuit(float points[][2], int length);

