/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  // vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int move() {
  m1.spin(forward, 100, percent);
  m1.spin(forward, 100, rpm);
  m2.spin(forward, 100, percent);
  m2.spin(forward, 100, rpm);
  m3.spin(forward, 100, percent);
  m3.spin(forward, 100, rpm);
  m4.spin(forward, 100, percent);
  m4.spin(forward, 100, rpm);

  wait (3000, msec);


  m1.spin(reverse, 100, percent);
  m1.spin(reverse, 100, rpm);
  m2.spin(reverse, 100, percent);
  m2.spin(reverse, 100, rpm);
  m3.spin(reverse, 100, percent);
  m3.spin(reverse, 100, rpm);
  m4.spin(reverse, 100, percent);
  m4.spin(reverse, 100, rpm);

  wait(300, msec);

  m1.spin(forward, 100, percent);
  m1.spin(forward, 100, rpm);
  m2.spin(reverse, 100, percent);
  m2.spin(reverse, 100, rpm);
  m3.spin(reverse, 100, percent);
  m3.spin(reverse, 100, rpm);
  m4.spin(forward, 100, percent);
  m4.spin(forward, 100, rpm);

  
  // m1.spin(forward, 100, percent);
  // m1.spin(forward, 100, rpm);
  // m2.spin(reverse, 100, percent);
  // m2.spin(reverse, 100, rpm);
  // m3.spin(reverse, 100, percent);
  // m3.spin(reverse, 100, rpm);
  // m4.spin(forward, 100, percent);
  // m4.spin(forward, 100, rpm);
  
  // wait(1000, msec);

  // m1.spinFor(forward, 500, msec, 100, percent);

  // m1.spin(forward, 100, percent);
  // m1.spin(forward, 100, rpm);
  // m2.spin(forward, 100, percent);
  // m2.spin(forward, 100, rpm);
  // m3.spin(forward, 100, percent);
  // m3.spin(forward, 100, rpm);
  // m4.spin(forward, 100, percent);
  // m4.spin(forward, 100, rpm);

  // wait (5000, msec);


  // m1.spin(reverse, 0, percent);
  // m1.spin(reverse, 0, rpm);
  // m2.spin(reverse, 0, percent);
  // m2.spin(reverse, 0, rpm);
  // m3.spin(reverse, 0, percent);
  // m3.spin(reverse, 0, rpm);
  // m4.spin(reverse, 0, percent);
  // m4.spin(reverse, 0, rpm);

  
  return 0;
}

void autonomous(void) {
  // ..........................................................................
  // Do whatever you need to do in the function above
  // ..........................................................................

  Brain.Screen.clearScreen();
  Brain.Screen.print("Comp");

  task m(move);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    vexcodeInit();
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
