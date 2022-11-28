#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen ( + other information)
brain Brain;

// CONFIGURATIONS: IF YOU DON'T KNOW WHAT YOU'RE DOING, ONLY MODIFY THE FOLLOWING LINES
unsigned int robotWidthMM = 0;
unsigned int wheelDiameterMM = 0;

auto m1MotorPort = PORT17;
auto m2MotorPort = PORT18;
auto m3MotorPort = PORT19;
auto m4MotorPort = PORT20;

auto motorRatio = ratio18_1;

int controllerJoystickMinActivation = 15; // Nothing will happen if the joystick is not moved beyond this value.
int controllerJoystickMaxBeforeFullPower = 90; // Full power applied if joystick is moved beyond this value.

int controllerAngleofDiscrepancyStartDelta = 35;
int controllerAxisSum = 170;


// Motors
motor m1 = motor(m1MotorPort, motorRatio, false);
motor m2 = motor(m2MotorPort, motorRatio, false);
motor m3 = motor(m3MotorPort, motorRatio, false);
motor m4 = motor(m4MotorPort, motorRatio, false);


// A global instance of MotorVector, to be reused every time the input needs to be processed
MotorVector vec = MotorVector(controllerAngleofDiscrepancyStartDelta, controllerAxisSum);


// Controller
controller PrimaryController = controller(primary);

task processInput;
task displayInput;
task displayStats;

// Initialize
void vexcodeInit(void) {
  // Start the necessary tasks
  displayStats = task(displaySystemStats);
  processInput = task(processControllerInput);
  displayInput = task(displayControllerInput);

  m1.resetPosition();
  m2.resetPosition();
  m3.resetPosition();
  m4.resetPosition();
}