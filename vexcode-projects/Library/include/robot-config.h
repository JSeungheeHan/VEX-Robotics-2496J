using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern inertial inert;
extern motor intakeL;
extern motor intakeR;
extern motor shooter;
extern motor top_right;
extern motor top_left;
extern motor bottom_right;
extern motor bottom_left;
extern rotation RotationL;
extern rotation RotationR;
extern motor roller;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );