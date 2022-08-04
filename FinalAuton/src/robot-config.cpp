#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial inert = inertial(PORT16);
motor intakeL = motor(PORT17, ratio18_1, false);
motor intakeR = motor(PORT18, ratio18_1, false);
motor shooter = motor(PORT2, ratio6_1, false);
motor top_right = motor(PORT8, ratio18_1, false);
motor top_left = motor(PORT1, ratio18_1, false);
motor bottom_right = motor(PORT20, ratio18_1, false);
motor bottom_left = motor(PORT11, ratio18_1, false);
rotation RotationL = rotation(PORT13, false);
rotation RotationR = rotation(PORT19, true);
motor roller = motor(PORT9, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}