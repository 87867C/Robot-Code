#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LF = motor(PORT1, ratio18_1, false);
motor LR = motor(PORT3, ratio18_1, false);
motor RF = motor(PORT2, ratio18_1, true);
motor RR = motor(PORT4, ratio18_1, true);
motor Lcl = motor(PORT7, ratio18_1, false);
motor Rcl = motor(PORT6, ratio18_1, true);
motor trans = motor(PORT20, ratio18_1, false);
motor shot = motor(PORT5, ratio18_1, true);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT13);
optical OpticalU = optical(PORT12);
optical OpticalD = optical(PORT11);

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