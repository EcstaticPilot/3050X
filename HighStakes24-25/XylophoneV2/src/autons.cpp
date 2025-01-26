#include "vex.h"
#include "stanley.h"
#include "autonSelector.h"
#include <vector>
using namespace vex;
extern competition Competition;
extern bool clamped;
extern timer clampTimer;
void testAuton()
{
  robot = point(-58.537, 11.258);
  gyro1.setRotation(65 * team, degrees);
  drive(-100, -100, 400);
  inchDrive(15);
  rotateTo(67 + 180);
  vex::thread([]()
              {
    waitUntil(clampDistance.objectDistance(mm)<=40 || clamp.value()==true);
    clamp.set(true);
    drive_brake(brake);
    return; });
  inchDrive(-38, 1, 2, 0.5);
  drive(-33, -33, 0);
  waitUntil(clampDistance.objectDistance(mm) <= 40 or clamp.value() == true);
  drive_brake(brake);
  // inchDrive(-15,0.75,5);
  clamp.set(true);
  clamped = true;
  wait(100, msec);
  rotateTo(0 * team);
  intake.spin(fwd, 12, volt);
  inchDrive(20, 2);
  rotateTo(100 * team);
  inchDrive(8, 2);
  wait(250, msec);
  inchDrive(-8, 2);
  rotateTo(70 * team);
  inchDrive(9, 2);
  wait(250, msec);
  inchDrive(-12, 2);
  rotateTo(160 * team);
  inchDrive(35);
  intake.stop(coast);
  return;
  gyro1.setRotation(295, degrees);
  gyro1.setHeading(295, degrees);
  inchDrive(-22);
  // waitUntil(clampOptical.isNearObject());
  clamped = true;
  clamp.set(true);
  drive_brake();
  intake.spin(fwd, 12, volt);
  rotateTo(0);
  inchDrive(20);
  wait(500, msec);
  rotateTo(75);
  inchDrive(12, 0.5);
  wait(2, sec);
  drive(25, -10, 1000);
  rotateTo(90);
  return;
  gyro1.setRotation(-90, degrees);
  robot = point(-53.985, -34.466);
  Stanley::setPath(std::vector<point>{
      {-53.985, -32.661},
      {-37.338, -32.259},
      {-20.891, -28.966},
      {-9.459, -40.483}});
  Stanley::run(reversed);
  drive_brake(brake);
  clamp.set(true);
}

void topSideAuton()
{
  gyro1.setRotation(65 * team + 180, degrees);
  robot = point(-60 * team, 10);
  vex::thread([]()
              {
    timer Timer;
    waitUntil(clampDistance.objectDistance(mm)<45 || clamp.value()==true);
    clamp.set(true);
    clamped=true;
    return; });
  inchDrive(-20, 1, 1.5, 1);
  drive(-30, -30, 0);
  waitUntil(clampDistance.objectDistance(mm) < 45 || clamp.value() == true);
  drive_brake(brake);
  clamp.set(true);
  clamped = true;
  wait(100, msec);
  // inchDrive(-15,0.75,5);

  // first ring
  rotateTo(robot.angleTo(point(-36 * team, 48)));
  intake.spin(fwd, 12, volt);
  inchDrive(20, 2);

  // ABORT!!!
  drive_brake();
  

  // next ring
  rotateTo(77 * team);
  //drive(75, 75, 0);
  inchDrive(12);
  drive_brake();
  wait(250, msec);
  inchDrive(-12, 2);

  
  // next ring
  rotateTo(64.5 * team);
  inchDrive(11.5);
  drive_brake(brake);
  wait(250, msec);

  inchDrive(-14, 2);
  rotateTo(160 * team);
  inchDrive(38, 3);
  intake.stop(coast);

  return;

  robot = point(-58.537, 11.258);
  gyro1.setRotation(45 * team, degrees);
  drive(-100, -100, 400);
  inchDrive(12);
  rotateTo((69.34706175776978 + 360) * team);
  inchDrive(-38.411);
  clamp.set(true);
  wait(100, msec);
  rotateTo(0);
  intake.spin(fwd, 12, volt);
  inchDrive(20);
  wait(500, msec);
  rotateTo(75 * team);
  inchDrive(12, 0.5);
  wait(1, sec);
  rotateTo(100 * team);
  wait(2, sec);
  inchDrive(-12);
  return;
  gyro1.setRotation(295 * team, degrees);
  gyro1.setHeading(295 * team, degrees);
  inchDrive(-24);
  clamp.set(true);
  drive_brake();
  intake.spin(fwd, 12, volt);
  rotateTo(0);
  inchDrive(20);
  wait(500, msec);
  rotateTo(75 * team);
  inchDrive(12, 0.5);
  wait(1, sec);
  rotateTo(100 * team);
  wait(2, sec);
  inchDrive(-12);
  rotateTo(90 * team);
  clamped = true;
}

void bottomSideAuton()
{

  gyro1.setRotation((-65) * team, degrees);
    vex::thread([]()
              {
    timer Timer;
    waitUntil(clampDistance.objectDistance(mm)<40 || clamp.value()==true);
    clamp.set(true);
    drive_brake(brake);
    clamped=true;
    return; });
    inchDrive(-30);

  drive(-30, -30, 0);
  waitUntil(clampDistance.objectDistance(mm) < 40 || clamp.value() == true);
  drive_brake();
  intake.spin(fwd, 12, volt);
  rotateTo(-170 * team);
  wait(500, msec);
  inchDrive(20, 2);
  wait(500, msec);
  rotateTo(30 * team);
  inchDrive(35);
  wait(10, sec);
  return;
  robot = point(58 * team, -11);

  gyro1.setRotation(180, degrees);
  inchDrive(-5, 2);
  inchDrive(12, 1.5);
  rotateTo((103 + 180) * team);
  vex::thread([]()
              {
    timer Timer;
    waitUntil(clampDistance.objectDistance(mm)<45 || clamp.value()==true);
    clamp.set(true);
    clamped=true;
    return; });
  inchDrive(-38, 1, 1.5, 1);
  drive(-30, -30, 0);
  waitUntil(clampDistance.objectDistance(mm) < 45 || clamp.value() == true);
  drive_brake(brake);
  clamp.set(true);
  clamped = true;
  wait(100, msec);
  // inchDrive(-15,0.75,5);

  // first ring
  rotateTo(190 * team);
  intake.spin(fwd, 12, volt);
  inchDrive(20, 2);
  wait(500, msec);
  rotateTo(15 * team);
  inchDrive(20, 2);
  return;
  rotateTo((180 + 45) * team);
  inchDrive(20);
  rotateTo(-110 * team);
  intake.spin(fwd, -12, volt);
  inchDrive(15, 1, 0.75);
  drive(50, 50, 1500);
  intake.spin(fwd, 12, volt);
  drive(50, 50, 750);
  // wait(1,sec);
  inchDrive(-12);
}

void skillsAuton()
{
  intake.spin(fwd, 12, volt);
  gyro1.setRotation(-60, degrees);
  robot = point(-57.796, -11.801);
  inchDrive(-5);
  clamp.set(true);
  rotateTo(-185);
  inchDrive(23.152);
  rotateTo(89.68709006668418);
  inchDrive(73.61);
  rotateTo(-112.52950498819195);
  inchDrive(29.314);
  rotateTo(-30.963523141388194);
  inchDrive(42.102);
  rotateTo(-123.2752389730212);
  inchDrive(38.384);
  rotateTo(158.83693051484934);
  inchDrive(13.334);
  rotateTo(0);
  clamp.set(false);
  rotateTo(180 + 11.381260163683647);

  inchDrive(-63.014);
  rotateTo(180 - 27.7841073761021);
  inchDrive(-21.084);
  clamp.set(true);
  rotateTo(0.758577069059452);
  inchDrive(30.288);
  rotateTo(110.18026758572817);
  inchDrive(25.001);
  rotateTo(88.02451467384027);
  inchDrive(46.559);
  rotateTo(179.56052269798298);
  inchDrive(26.075);
  rotateTo(180 - 65.56686219655735);
  clamp.set(false);
  inchDrive(-86.797);

  return;
  intake.spin(fwd, 12, volt);
  gyro1.setRotation(-60, degrees);
  robot = point(-58.598, -9.996);
  inchDrive(-15.907);
  clamp.set(true);
  Stanley::setPath(std::vector<point>{
      {-47.968, -21.83},
      {-59.4, -10.397},
      {-29.917, 10.462},
      {-23.298, 24.301},
      {-23.298, 24.301},
      {-16.679, 38.14},
      {-23.097, 36.536},
      {-23.699, 48.168},
      {-23.699, 48.168},
      {-24.301, 59.801},
      {-13.671, 57.996},
      {-0.233, 58.197},
      {-0.233, 58.197},
      {40.683, 60.403},
      {12.804, 34.931},
      {23.635, 24.903},
      {23.635, 24.903},
      {37.875, 10.863},
      {31.056, -20.426},
      {23.234, -23.836}});
  Stanley::run(slow);
  clamp.set(false);
  Stanley::setPath(std::vector<point>{
      {23.234, -23.836},
      {31.457, -20.827},
      {40.483, -9.395},
      {46.7, 0.834}});
  Stanley::run(reversed);
  clamp.set(true);
  Stanley::setPath(std::vector<point>{
      {46.7, 0.834},
      {39.48, -30.655},
      {33.663, -38.276},
      {22.432, -47.503},
      {22.432, -47.503},
      {16.013, -51.514},
      {12.604, -60.138},
      {0.77, -59.136},
      {0.77, -59.136},
      {-11.063, -58.133},
      {-14.072, -51.915},
      {-24.301, -47.101},
      {-24.301, -47.101},
      {-34.53, -42.288},
      {-41.349, -46.6},
      {-59.4, -46.7}});
  Stanley::run(slow);
  rotateTo(138.2075487987951);
  inchDrive(17.756);
}