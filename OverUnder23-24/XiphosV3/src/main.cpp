/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Nicool Ramanuja                                           */
/*    Created:      9/3/2023, 5:15:15 PM                                      */
/*    Description:  program for palos verdes peninsula high school team 3050X */
/*                  robot version coorect version                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "comp_debug.h"
#include "sylib/sylib.hpp"
#include "autonSelector.h"
#include "gifclass.h"
using namespace vex;

// extern sylib::Addrled LED;

//  A global instance of competition
competition Competition;

// debug class
// competition_debug Cdebug(Competition);

bool LED_ENABLE = false;
const bool isCordy = false;
void disableWatcher();
// piston stuff
void toggle(pneumatics *piston)
{
	piston->set(!(piston->value()));
}
vex::turnType currWing = left;
bool blocker = false;
bool wings = false;
bool prevBlocker = blocker;
bool prevWings = wings;
void LEDsOn()
{
	LED_ENABLE = true;
	prevWings = !wings;
	prevBlocker = !blocker;
}
void toggleLEDs()
{
	std::cout << "toggle LED";
	LED_ENABLE = !LED_ENABLE;
	if (LED_ENABLE)
	{
		LEDsOn();
	}
}
void intake()
{
	intakeMotor.spin(fwd, 12, volt);
}
void outtake()
{
	intakeMotor.spin(reverse, 12, volt);
}
void flapWings()
{

	toggle(&wingL);
	toggle(&wingR);
	wings = !wings;
}

void flapLift()
{
	toggle(&wingL);
	toggle(&wingR);
	wings = !wings;
}

// devices stuff
bool devicesCheck()
{
	if (LF.installed() && RF.installed() && LB.installed() && RB.installed() && LM.installed() && RM.installed() &&
		gyro1.installed() && intakeMotor.installed())
	{
		return true;
	}
	else
	{
		return false;
	}
}

// thread for printing on the controller
int ControllerPrint()
{
	while (true)
	{

		Controller1.Screen.setCursor(1, 1);
		Controller1.Screen.print("pos= (%.1f  , %.1f  )       ", robot.x, robot.y);
		Controller1.Screen.setCursor(2, 1);
		Controller1.Screen.print("%f", gyro1.yaw());
		Controller1.Screen.setCursor(3, 1);

		if (devicesCheck())
		{
			Controller1.Screen.print("All Devices Connected :]       ");
		}
		else
		{
			// Controller1.rumble(".");
			if (!LF.installed())
			{
				Controller1.Screen.print("LF not connected");
			}
			else if (!RF.installed())
			{
				Controller1.Screen.print("RF not connected");
			}
			else if (!LB.installed())
			{
				Controller1.Screen.print("LB not connected");
			}
			else if (!RB.installed())
			{
				Controller1.Screen.print("RB not connected");
			}
			else if (!gyro1.installed())
			{
				Controller1.Screen.print("gyro not connected");
			}
			else if (!LM.installed())
			{
				Controller1.Screen.print("LM not connected");
			}
			else if (!RM.installed())
			{
				Controller1.Screen.print("RM not connected");
			}
			else
			{
				Controller1.Screen.print("unknown device not connected");
			}

			this_thread::sleep_for(500);
		}

		this_thread::sleep_for(500);
	}
}
/*

██╗       ███████╗  ██████╗   ███████╗
██║       ██╔════╝  ██╔══██╗  ██╔════╝
██║       █████╗    ██║  ██║  ███████╗
██║       ██╔══╝    ██║  ██║  ╚════██║
███████╗  ███████╗  ██████╔╝  ███████║
╚══════╝  ╚══════╝  ╚═════╝   ╚══════╝

*/

int LEDmanager(void)
{

	sylib::Addrled wingsLED1 = sylib::Addrled(22, 6, 35);
	sylib::Addrled wingsLED2 = sylib::Addrled(22, 8, 35);

	// sylib::Addrled blockerLED1 = sylib::Addrled(22, 2, 35);
	// sylib::Addrled blockerLED2 = sylib::Addrled(19, 2, 35);
	int wingColor = 0xFF0000;
	if (LED_ENABLE)
	{
		wingsLED1.rainbow(5);
		wingsLED2.rainbow(5);
	}

	while (true)
	{
		if (LED_ENABLE)
		{
			// wings
			if (wings != prevWings)
			{
				prevWings = wings;
				if (wings)
				{
					wingsLED1.pulse(wingColor, 36, 20, 0, false, 0);
					wingsLED2.pulse(wingColor, 36, 20, 0, false, 0);
					wingsLED1.set_all(wingColor);
					wingsLED2.set_all(wingColor);
				}
				else
				{
					wingsLED1.rainbow(5);
					wingsLED2.rainbow(5);
				}
			}
		}
		else
		{
			wingsLED1.turn_off();
			wingsLED2.turn_off();
		}
		this_thread::sleep_for(50);
	}
	return 0;
}
/*
-------------------------------------------------------------------------------------------------

*	██████╗   ██████╗   ███████╗           █████╗   ██╗   ██╗  ████████╗   ██████╗   ███╗   ██╗
*	██╔══██╗  ██╔══██╗  ██╔════╝          ██╔══██╗  ██║   ██║  ╚══██╔══╝  ██╔═══██╗  ████╗  ██║
*	██████╔╝  ██████╔╝  █████╗    █████╗  ███████║  ██║   ██║     ██║     ██║   ██║  ██╔██╗ ██║
*	██╔═══╝   ██╔══██╗  ██╔══╝    ╚════╝  ██╔══██║  ██║   ██║     ██║     ██║   ██║  ██║╚██╗██║
*	██║       ██║  ██║  ███████╗          ██║  ██║  ╚██████╔╝     ██║     ╚██████╔╝  ██║ ╚████║
*	╚═╝       ╚═╝  ╚═╝  ╚══════╝          ╚═╝  ╚═╝   ╚═════╝      ╚═╝      ╚═════╝   ╚═╝  ╚═══╝

-------------------------------------------------------------------------------------------------
*/

void pre_auton(void)
{
	sylib::initialize();

	// draw field and initailization of buttons
	initializeAutonSelector();

	gyro1.calibrate();
	//? should we wait? I dont think its necessary
	/*
	while(gyro1.isCalibrating())
	{
		wait(10,msec);
	}
	*/

	if (Brain.Battery.capacity(pct) <5)
	{
		Controller1.rumble("---");
		Controller1.Screen.print("low battery");
	}
	// launch threads
	//? maybe these could be tasks instead of threads that get stopped between mode and reintialized
	thread ControllerPrinting = thread(ControllerPrint);
	ControllerPrinting.setPriority(1);
	thread posTrack = thread(odometery);
	thread LEDcontrol = thread(LEDmanager);

	if (devicesCheck())
	{
		// Brain.Screen.print("Devices Connected");
	}
	else
	{
		Controller1.rumble("....");
		Brain.Screen.print("Devices Not Connected");
	}
}

/*
---------------------------------------------------------------------------

!		 █████╗   ██╗   ██╗  ████████╗   ██████╗   ███╗   ██╗
!		██╔══██╗  ██║   ██║  ╚══██╔══╝  ██╔═══██╗  ████╗  ██║
!		███████║  ██║   ██║     ██║     ██║   ██║  ██╔██╗ ██║
!		██╔══██║  ██║   ██║     ██║     ██║   ██║  ██║╚██╗██║
!		██║  ██║  ╚██████╔╝     ██║     ╚██████╔╝  ██║ ╚████║
!		╚═╝  ╚═╝   ╚═════╝      ╚═╝      ╚═════╝   ╚═╝  ╚═══╝

---------------------------------------------------------------------------
*/
// auton functions
void offensiveZoneAuton();
void offensiveZoneAuton2();
void defensiveZoneAuton();
void defensiveZoneAuton2();
void skillsAuton();
void testAuton();
void autonomous(void)
{

	if (Competition.isFieldControl() || Competition.isCompetitionSwitch())
		LEDsOn();
	std::cout << selectedAuton;
	waitUntil(!gyro1.isCalibrating());
	switch (selectedAuton)
	{
	case offensiveZone:
	{
		offensiveZoneAuton();
		break;
	}
	case offensiveZoneRush:
	{
		offensiveZoneAuton2();
		break;
	}
	case defensizeZoneRush:{
		defensiveZoneAuton2();
		break;
	}
	case defensiveZone:
	{
		defensiveZoneAuton();
		break;
	}

	case skills:
	{
		skillsAuton();
		break;
	}
	case test:
	{
		testAuton();
		break;
	}
	}
	drive_brake(brake);
	if (LED_ENABLE)
		toggleLEDs();
	return;
}

/*
---------------------------------------------------------------------------

?		██████╗   ██████╗   ██╗  ██╗   ██╗  ███████╗  ██████╗
?		██╔══██╗  ██╔══██╗  ██║  ██║   ██║  ██╔════╝  ██╔══██╗
?		██║  ██║  ██████╔╝  ██║  ██║   ██║  █████╗    ██████╔╝
?		██║  ██║  ██╔══██╗  ██║  ╚██╗ ██╔╝  ██╔══╝    ██╔══██╗
?		██████╔╝  ██║  ██║  ██║   ╚████╔╝   ███████╗  ██║  ██║
?		╚═════╝   ╚═╝  ╚═╝  ╚═╝    ╚═══╝    ╚══════╝  ╚═╝  ╚═╝

---------------------------------------------------------------------------
*/
void driverSkillsAuton();
void usercontrol()
{	
	bool bskills=false;
	wingL.set(false);
	wingR.set(false);
	if (Competition.isFieldControl() || Competition.isCompetitionSwitch())
		LEDsOn();

	if ((Competition.isFieldControl() || Competition.isCompetitionSwitch()) && selectedAuton == skills)
	{
		bskills=true;
		driverSkillsAuton();
	}
	std::cout << "start driver" << std::endl;
	thread hang(disableWatcher);

	blocker = false;
	wings = false;
	brakeType driveBrake = coast;
	brakeType intakeBrake = coast;
	// cataRot.setPosition(30, degrees);
	//  bool LEDon = false;
	//   User control code here, inside the loop
	Brain.resetTimer();
	// Brain.Screen.clearScreen();
	// wait(1,sec);
	// vex::Gif gif("hollywood.gif", 26, 0);
	/*gif list
	capyinbucket
	capybara
	tias
	ciaran-moment
	hollywood
	socal-alliance
	triball\
	world
	*/
	Optical.setLightPower(100, pct);
	while (true)
	{
		if (Optical.isNearObject())
		{
			Optical.setLight(ledState::on);
		}
		else
		{
			Optical.setLight(ledState::off);
		}
		if (Controller1.ButtonUp.pressing())
		{
			// wings = true;
			lift1.set(true);
			lift2.set(true);
		}
		else if (Controller1.ButtonDown.pressing())
		{
			// wings = false;
			lift1.set(false);
			lift2.set(false);
		}
		if(bskills and Controller1.ButtonA.pressing()){
			wingR.set(true);
		}
		else if (Controller1.ButtonL2.pressing())
		{

			wings = true;
			if (selectedAuton != skills)
			{
				if (currWing == left)
				{
					wingL.set(true);
				}
				else if (currWing == right)
				{
					wingR.set(true);
				}
			}
			else
			{
				wingL.set(true);
				wingR.set(true);
			}
		}
		else
		{
			wings = false;
			wingL.set(false);
			wingR.set(false);
		}

		if (Controller1.ButtonL1.pressing())
		{
			wings = true;
			wingFL.set(true);
			wingFR.set(true);
		}
		else
		{
			wings = false;
			wingFL.set(false);
			wingFR.set(false);
		}

		if (Controller1.ButtonY.pressing())
		{
			cata.stop(coast);
		}
		else if (Controller1.ButtonX.pressing())
		{
			cata.spin(fwd,100,rpm);
			driveBrake = hold;
		}
		else
		{
			cata.stop(hold);
			driveBrake = coast;
		}

		// std::cout<<cata.power()<<std::endl;
		// intakeco
		if (Controller1.ButtonR1.pressing())
		{
			intakeMotor.spin(fwd, 12, volt);
			intakeBrake = hold;
		}

		// outake
		else if (Controller1.ButtonR2.pressing())
		{
			intakeMotor.spin(fwd, -12, volt);
			intakeBrake = coast;
		}

		// stop the intake
		else
		{
			intakeMotor.stop(intakeBrake);
		}

		// deadzone
		if ((abs(Controller1.Axis3.position(pct)) < 5) and (abs((isCordy ? Controller1.Axis1.position(pct) : Controller1.Axis2.position(pct))) < 5))
		{
			drive_brake(driveBrake);
		}
		else
		{
			if (!isCordy)
			{
				voltDrive(Controller1.Axis3.position(), Controller1.Axis2.position());
			}
			else
			{
				voltDrive(Controller1.Axis3.position() + Controller1.Axis1.position(), Controller1.Axis3.position() - Controller1.Axis1.position());
			}
		}
		std::cout << robot.x << ", " << robot.y << std::endl;
		// std::cout <<Controller1.Axis3.position()<<","<<Controller1.Axis2.position()<< std::endl;
		wait(10, msec);
	}
}
/*

███╗   ███╗   █████╗   ██╗  ███╗   ██╗
████╗ ████║  ██╔══██╗  ██║  ████╗  ██║
██╔████╔██║  ███████║  ██║  ██╔██╗ ██║
██║╚██╔╝██║  ██╔══██║  ██║  ██║╚██╗██║
██║ ╚═╝ ██║  ██║  ██║  ██║  ██║ ╚████║
╚═╝     ╚═╝  ╚═╝  ╚═╝  ╚═╝  ╚═╝  ╚═══╝

*/
void endCode()
{
	vexSystemExitRequest();
}
// https://www.fancytextpro.com/BigTextGenerator/Big
int main()
{
	std::cout << "yes";
	// Controller1.ButtonRight.pressed(toggleLEDs);
	//  Controller1.ButtonL1.pressed(flapWings);
	Brain.Screen.pressed(onScreenPress);
	Controller1.ButtonRight.pressed([]()
									{ currWing = right; });
	Controller1.ButtonLeft.pressed([]()
								   { currWing = left; });
	//  Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(autonomous);
	Competition.drivercontrol(usercontrol);

	// Run the pre-autonomous function.
	pre_auton();

	// Prevent main from exiting with an infinite loop.
	while (true)
	{
		wait(100, msec);
	}
}
