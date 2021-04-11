using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LF;
extern motor LR;
extern motor RF;
extern motor RR;
extern motor Lcl;
extern motor Rcl;
extern motor trans;
extern motor shot;
extern controller Controller1;
extern inertial Inertial;
extern optical OpticalU;
extern optical OpticalD;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );