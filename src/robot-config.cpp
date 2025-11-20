#include "vex.h"

#include "robot-config.h"
#include <new>
#include <cstring>

using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller master = controller(primary);
inertial imu = inertial(PORT16);

//sensors
// digital_in limit_switch = digital_in(Brain.ThreeWirePort.D);

//drivetrain
motor LeftFront = motor(PORT5, ratio6_1, true);
motor LeftMiddle = motor(PORT4, ratio6_1, false); 
motor LeftBack = motor(PORT6, ratio6_1, true);
    
motor RightFront = motor(PORT2, ratio6_1, false);
motor RightMiddle = motor(PORT3, ratio6_1, true); 
motor RightBack = motor(PORT1, ratio6_1, false);




motor_group RightDrive = motor_group(RightFront, RightMiddle, RightBack);
motor_group LeftDrive = motor_group(LeftFront, LeftMiddle, LeftBack);

// other motors
motor top_roller = motor(PORT18, ratio6_1, false);
motor intake = motor(PORT9, ratio6_1, true);

// pneumatics
digital_out matchloader = digital_out(Brain.ThreeWirePort.A);
digital_out guard = digital_out(Brain.ThreeWirePort.B);
digital_out middlegoal = digital_out(Brain.ThreeWirePort.D);


// VEXcode device constructors

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}