using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller controller_1;
extern motor left_f;
extern motor left_m;
extern motor left_b;
extern motor_group left_chassis;

extern motor right_f;
extern motor right_m;
extern motor right_b;
extern motor_group right_chassis;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );