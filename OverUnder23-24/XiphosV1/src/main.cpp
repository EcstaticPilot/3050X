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

// piston stuff
void toggle(pneumatics *piston)
{
	piston->set(!(piston->value()));
}

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
void toggleBlocker()
{
	toggle(&blockerL);
	toggle(&blockerR);
	blocker = !blocker;
}
void flapWings()
{
	toggle(&wingL);
	toggle(&wingR);
	wings = !wings;
}

// devices stuff
bool devicesCheck()
{
	if (LF.installed() && RF.installed() && LB.installed() && RB.installed() && LM.installed() && RM.installed() &&
		gyro1.installed() && cata.installed() && intake.installed() && cataRot.installed())
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
		Controller1.Screen.print("%f", gyro1.angle(degrees));
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

			wait(250, msec);
		}

		this_thread::sleep_for(200);
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
	
	sylib::Addrled wingsLED1 = sylib::Addrled(19, 1, 35);
	sylib::Addrled wingsLED2 = sylib::Addrled(22, 1, 35);

	sylib::Addrled blockerLED1 = sylib::Addrled(22, 2, 35);
	sylib::Addrled blockerLED2 = sylib::Addrled(19, 2, 35);

	int blockerColor = 0xFF0000;
	int wingColor = 0xFF0000;
	if (LED_ENABLE)
	{
		blockerLED1.rainbow(5);
		blockerLED2.rainbow(5);
		wingsLED1.rainbow(5);
		wingsLED2.rainbow(5);
	}
	uint32_t  red1 = red.rgb();
	uint32_t grey = ClrDarkGray;
	
	
	
	while (true)
	{
		if (LED_ENABLE)
		{
			// blocker
			if (blocker != prevBlocker)
			{
				prevBlocker = blocker;
				if (blocker)
				{
					blockerLED1.pulse(blockerColor, 36, 20, 0, false, 0,false);
					blockerLED2.pulse(blockerColor, 36, 20, 0, false, 0,false);
					blockerLED1.set_all(blockerColor);
					blockerLED2.set_all(blockerColor);
				}
				else
				{
					blockerLED1.rainbow(5);
					blockerLED2.rainbow(5);
				}
			}
			// wings
			if (wings != prevWings)
			{
				prevWings = wings;
				if (wings)
				{
					wingsLED1.pulse(wingColor, 36, 20, 0, false, 0,false);
					wingsLED2.pulse(wingColor, 36, 20, 0, false, 0,false);
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
			blockerLED1.turn_off();
			blockerLED2.turn_off();
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

	if (Brain.Battery.capacity(pct) < 25)
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
void defensiveZoneAuton();
void skillsAuton();
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

void usercontrol()
{
	if (Competition.isFieldControl() || Competition.isCompetitionSwitch())
		LEDsOn();
	std::cout << "start driver" << std::endl;

	blockerL.set(false);
	blockerR.set(false);
	wingL.set(false);
	wingR.set(false);
	blocker=false;
	wings=false;
	brakeType driveBrake = coast;
	brakeType intakeBrake = coast;
	cataRot.setPosition(30, degrees);
	// bool LEDon = false;
	//  User control code here, inside the loop
	Brain.resetTimer();
	//Brain.Screen.clearScreen();
	//wait(1,sec);
	vex::Gif gif("capyinbucket.gif", 26, 0);

	while (true)
	{
		
		// std::cout<<cataRot.position(degrees)<<std::endl;
		if (Controller1.ButtonL1.pressing())
		{
			wings = true;
			wingL.set(true);
			wingR.set(true);
		}
		else
		{
			wings = false;
			wingL.set(false);
			wingR.set(false);
		}

		// cata manager
		if (Controller1.ButtonY.pressing()){
			cata.spin(fwd, -6, volt);
		}
		else if (Controller1.ButtonX.pressing() || cataRot.position(degrees) < 55)
		{
			cata.spin(fwd, 12 * 0.85, volt);
			if (Controller1.ButtonX.pressing())
			{
				driveBrake = hold;
			}
			else
			{
				driveBrake = coast;
			}
		}
		else
		{
			cata.stop(hold);
			driveBrake = coast;
		}

		// intake
		if (Controller1.ButtonR1.pressing())
		{
			intake.spin(fwd, 12, volt);
			intakeBrake = hold;
		}

		// outake
		else if (Controller1.ButtonR2.pressing())
		{
			intake.spin(fwd, -12, volt);
			intakeBrake = coast;
		}

		// stop the intake
		else
		{
			intake.stop(intakeBrake);
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
// https://www.fancytextpro.com/BigTextGenerator/Big
int main()
{

	Controller1.ButtonDown.pressed(toggleLEDs);
	Controller1.ButtonB.pressed(toggleBlocker);
	Brain.Screen.pressed(onScreenPress);
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
