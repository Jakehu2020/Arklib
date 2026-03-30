#include "vex.h"

using namespace vex;

brain Brain; // gloabl brain instance

// VEXcode device constructors
controller controller_1 = controller(primary);

/* 
    PERMANENT devices (Need a defintion for Arklib)
    - Drivetrain motors can be deleted
      (Each chassis motor group must contain at least one motor)
    - Used for unity throughout the project
*/
motor left_f = motor(PORT1, ratio6_1, true);
motor left_m = motor(PORT2, ratio6_1, true);
motor left_b = motor(PORT3, ratio6_1, true);
motor_group left_chassis = motor_group(left_f, left_m, left_b);

motor right_f = motor(PORT4, ratio6_1, true);
motor right_m = motor(PORT5, ratio6_1, true);
motor right_b = motor(PORT6, ratio6_1, true);
motor_group right_chassis = motor_group(right_f, right_m, right_b);

/*
    USER devices
*/