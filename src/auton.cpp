#include "vex.h"

#include "../include/Hurr-Template/s-drive.h"

void odom_constants(){
  default_constants();
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
}

void s_drive_test(){
  default_constants();

  using namespace sdrive;

  // s_drive(distance)

  // s_drive(distance, heading);

 // s_drive_shortest(48, 12);  // find shortest path to x,y

  // s_drive can also accept a bonus parameter for degrees e.g what degree you want the robot to be when 
  //it has finished the motion
  

  // s_drive(10); 

  // s_drive(10, 90); 


  // 9 block auton
  spinIntake(1);

  s_drive(60);

  s_drive(-35, 200);

  chassis.turn_to_angle(-90);

  s_drive(-15);

  toggleGuard();

  spinTopRoller(1);

  wait(2.5, sec);

  stopTopRoller();

  toggleMatchLoader();

  s_drive(12);

  wait(1, sec);

  s_drive(-12);

  spinTopRoller(1);

  toggleMatchLoader();


  


  
  

  
}

void s_drive_chain_test() {
  default_constants();

  using namespace sdrive;

  s_drive(30, 280); 
}

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0,50, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(0,70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0,110, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(0,130, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_point(0, 18, 90);
  chassis.holonomic_drive_to_point(18, 0, 180);
  chassis.holonomic_drive_to_point(0, 18, 270);
  chassis.holonomic_drive_to_point(0, 0, 0);
}

void default_constants(){
  chassis.set_drive_constants(9, 0.8, 0.03, 0.8, 3);
  chassis.set_heading_constants(6, 0.3, 0.03, 5, 3);

  chassis.set_turn_constants(9, 0.30, 0.03, 1.56, 15);
  chassis.set_swing_constants(9, 0.30, 0.05, 2, 15);

  chassis.set_drive_exit_conditions(2, 200, 4000);
  chassis.set_turn_exit_conditions(1, 200, 1500);
  chassis.set_swing_exit_conditions(1, 200, 1500);
}

void test_functions (){
  default_constants();
  chassis.set_coordinates(0, 0, 0);

  chassis.drive_distance(30);
}

void right_side_9 (){
  default_constants();
  chassis.set_coordinates(0, 0, 0);

}

void prog_skills (){
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  
}