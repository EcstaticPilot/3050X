/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Nikhil Ramanuja                                           */
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
#include "monteCarlo.h"
#include <iostream>
using namespace vex;

// extern sylib::Addrled LED;

//  A global instance of competition
competition Competition;

// debug class
// competition_debug Cdebug(Competition);

bool LED_ENABLE = true;
timer clampTimer = timer();
bool clamped = false;
// piston stuff
void toggle(pneumatics *piston)
{
    piston->set(!(piston->value()));
}

void LEDsOn()
{
    LED_ENABLE = true;
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
// devices stuff
bool devicesCheck()
{
    if (LF.installed() && RF.installed() && LB.installed() && RB.installed() && LM.installed() && RM.installed() &&
        gyro1.installed())
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
        Controller1.Screen.print("%f", GPS.heading());
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("%.1f  , %.1f  , %.1f     " , leftDistance.objectDistance(inches),clampDistance.objectDistance(inches),  rightDistance.objectDistance(inches));
        
        /*if (devicesCheck() or true)
        {
            Controller1.Screen.print("%d         ", GPS.quality());
            // Controller1.Screen.print("All Devices Connected :]       ");
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
        */
        this_thread::sleep_for(100);
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
    // declare LEDS
    sylib::Addrled LED3 = sylib::Addrled(22, 4, 35);
    sylib::Addrled LED4 = sylib::Addrled(22, 5, 35);
    sylib::Addrled LED1 = sylib::Addrled(22, 6, 35);
    sylib::Addrled LED2 = sylib::Addrled(22, 8, 35);

    // cycle speed
    int speed = 10;

    // if we are enabled, start the LEDs rainbow

    if (LED_ENABLE == false)
    {
        LED1.turn_off();
        LED2.turn_off();
        LED3.turn_off();
        LED4.turn_off();
    }
    LED1.rainbow2(speed, true);
    LED2.rainbow2(speed);
    LED3.rainbow2(speed);
    LED4.rainbow2(speed, true);
    while (true)
    {
        // std::cout << clamp.value() << " , " << clampTimer.time() << std::endl;
        //  if LEDs are on and we are disabled, show dim team colors
        //  this is the ONLY thing that should be trigger repeatedly
        if (Competition.isEnabled() == false && LED_ENABLE)
        {
            if (team == tred)
            {
                LED1.set_all(0xf00000);
                LED2.set_all(0xf00000);
                LED3.set_all(0xf00000);
                LED4.set_all(0xf00000);
            }
            else if (team == tblue)
            {
                LED1.set_all(0x0000ff);
                LED2.set_all(0x0000ff);
                LED3.set_all(0x0000ff);
                LED4.set_all(0x0000ff);
            }
        }
        // if we are not disabled we are enabled. This shoulldn't trigger mutliple times so we check for control mode
        // Normally, it is on cycle but if we just got re enabled it will be manual, this checks for when the robot gets enabled at the start of auton or driver
        else if (LED_ENABLE && LED1.getControlMode() == sylib::SylibAddrledControlModeMANUAL)
        {
            LED1.rainbow2(speed, true);
            LED2.rainbow2(speed);
            LED3.rainbow2(speed);
            LED4.rainbow2(speed, true);
        }

        // this is for turning yellow from the clamp, we detecte this with the clamp timer, which is how long since the clamp was last used
        // we check the clamp value to make sure that we clamped instead of releaasing
        // again we check for LED enable

        else if (clampTimer.time() < 500 && LED_ENABLE)
        {
            if (clamped)
            {
                // save the buffer, this is to make it look consistent
                auto buffer1 = *LED1;
                auto buffer2 = *LED2;
                auto buffer3 = *LED3;
                auto buffer4 = *LED4;

                // set all LEDs to bright yellow
                uint32_t color = 0x968a00;
                LED1.set_all(color);
                LED2.set_all(color);
                LED3.set_all(color);
                LED4.set_all(color);
                wait(250, msec);
                LED1.set_all(0);
                LED2.set_all(0);
                LED3.set_all(0);
                LED4.set_all(0);
                wait(100, msec);
                LED1.set_all(color);
                LED2.set_all(color);
                LED3.set_all(color);
                LED4.set_all(color);
                wait(250, msec);
                LED1.set_all(0);
                LED2.set_all(0);
                LED3.set_all(0);
                LED4.set_all(0);
                wait(100, msec);
                LED1.set_all(color);
                LED2.set_all(color);
                LED3.set_all(color);
                LED4.set_all(color);
                wait(250, msec);

                // wait
                // std::cout<<"go back"<<std::cout;
                // put the saved buffer back and cycle
                LED1.cycle(buffer1, speed, 0, true);
                LED2.cycle(buffer2, speed);
                LED3.cycle(buffer3, speed);
                LED4.cycle(buffer4, speed, 0, true);
                wait(100, msec);
            }
        }

        // Main LED enable and disable
        // again, we only need to trigger this once
        if (LED_ENABLE and LED1.getControlMode() == sylib::SylibAddrledControlModeOFF)
        {
            // turn them all on
            LED1.turn_on();
            LED2.turn_on();
            LED3.turn_on();
            LED4.turn_on();

            // this is a bit risky, since we are using the saved buffer
            // if it got clear without being reset, they would not work
            // we may need to replace this with rainbow2 for safety
            LED1.cycle(*LED1, speed, 0, true);
            LED2.cycle(*LED2, speed);
            LED3.cycle(*LED3, speed);
            LED4.cycle(*LED4, speed, 0, true);
        }

        // if LED enable is false and the LEDs are not off, turn them off
        else if (!LED_ENABLE and LED1.getControlMode() != sylib::SylibAddrledControlModeOFF)
        {
            LED1.turn_off();
            LED2.turn_off();
            LED3.turn_off();
            LED4.turn_off();
        }
        this_thread::sleep_for(25);
    }
    return 1;
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

    if (Brain.Battery.capacity(pct) < 40)
    {
        Controller1.rumble("------");
        Controller1.Screen.print("low battery");
    }
    // launch threads
    //? maybe these could be tasks instead of threads that get stopped between mode and reintialized
    thread ControllerPrinting = thread(ControllerPrint);
    ControllerPrinting.setPriority(1);
    //thread posTrack = thread(odometeryThread);
   thread MCL = thread(MonteCarlo::thread);
    thread LEDcontrol = thread(LEDmanager);
    //thread MCL = thread(MonteCarlo::thread);
    if (devicesCheck())
    {
        // Brain.Screen.print("Devices Connected");
    }
    else
    {
        // Controller1.rumble("....");
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
void topSideAuton();
void bottomSideAuton();
void skillsAuton();
void testAuton();
void autonomous(void)
{
    if (Competition.isFieldControl())
        LEDsOn();
    std::cout << selectedAuton;
    waitUntil(!gyro1.isCalibrating());
    switch (selectedAuton)
    {
    case topSide:
    {
        topSideAuton();
        break;
    }
    case bottomSide:
    {
        bottomSideAuton();
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

    brakeType driveBrake = coast;

    // wait(1,sec);
    // vex::Gif gif("ciaran-moment.gif", 26, 0);
    /*gif list
    capyinbucket
    capybara
    tias
    ciaran-moment
    hollywood
    socal-alliance
    triball
    world
    jesscookie
    */
    while (true)
    {

        // intake code
        if (Controller1.ButtonR1.pressing())
        {
            intake.spin(fwd, 12, volt);
        }

        // outake
        else if (Controller1.ButtonR2.pressing())
        {
            intake.spin(fwd, -12, volt);
        }
        else if(Controller1.ButtonL1.pressing()){
            intakeMotor2.spin(fwd,12,volt);
            intakeMotor1.spin(fwd,-12,volt);
        }
        // stop the intake
        else
        {
            intake.stop(coast);
        }

        // CLAMP
        if (clampDistance.installed() and
            clampDistance.objectDistance(mm) < 35 and
            not clamp.value() and
            not(Controller1.ButtonA.pressing() or Controller1.ButtonB.pressing()) and
            clampTimer.time() > 750)
        {
            clamped = true;
            clampTimer.reset();
            clamp.set(true);
            Controller1.rumble(".");
        }

        // deadzone
        if ((abs(Controller1.Axis3.position(pct)) < 5) and (abs((Controller1.Axis2.position(pct))) < 5))
        {
            drive_brake(driveBrake);
        }
        else
        {
            voltDrive(Controller1.Axis3.position(), Controller1.Axis2.position());
        }
        // std::cout << robot.x << ",," << robot.y << std::endl;
        vex::wait(10, msec);
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
// https://google.github.io/styleguide/cppguide.html
int main()
{
    //! COMPETITION CALLBACKS
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Stuff for driver
    clampTimer = timer();

    Controller1.ButtonA.pressed([]()
                                {
        clamp.set(!clamp.value());
        clamped = !clamped;
        clampTimer.clear(); });
    Controller1.ButtonUp.pressed([](){
        armA.set(!armA.value());
        armB.set(!armB.value());
    });
    /* clampOptical.objectDetectThreshold(254);
     clampOptical.objectDetected([]()
                                 {
         if(!clamp.value()&&!Controller1.ButtonA.pressing() && clampTimer.time()>500 && Competition.isAutonomous() == false){
             clamped = true;
             clampTimer.reset();
             clamp.set(true);
             Controller1.rumble(".");
         } });
 */
    Brain.Screen.pressed(onScreenPress);

    Controller1.ButtonRight.pressed(toggleLEDs);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        vex::wait(100, msec);
    }
}
