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

  //   s_drive_chain({
  //   Command(32, 90),   // drive 32", curve to 90°
  //   Command(-20, 90),  // immediately back 20", keep 90°
  //   Command(15)        // then 15" forward, keep current angle
  // });


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

void hurrs_auton() {

  chassis.set_coordinates(0,0,0);

  default_constants();

  using namespace sdrive;

  // { 

  toggleMatchLoader();

  spinIntake(1);

  s_drive(32, 90);

  wait(.25, sec);  /* ===========   Drive Forward while turning 90 degrees and get matchload then score in top goal */

  spinTopRoller(1);

  s_drive(-16);

  stopTopRoller();

  stopIntake();

  // }

  // {

  toggleMatchLoader();

  s_drive(10, 160);

  spinIntake(1);

  s_drive(20, 180);

  s_drive(40, 200);

  toggleMiddleGoal();       // ======= Curve to three blocks parallel to top goal and intake then back up curve and 
 
  spinTopRoller(1);        // ======= drive straight while intaking other three and curve back score in middle goal

  s_drive(-16);            // Might swap the back up for just drive straight and correct angle to 180 deg but idk

  wait(.25, sec);          // Zuhayr or Jaden if ur reading this.... felt cute might delete later lol

  stopTopRoller();

  toggleMatchLoader();

  toggleMiddleGoal();

  // }

  // {

  s_drive(32, 90);

  wait(.25, sec);

  spinTopRoller(1);

  s_drive(-20);        /* ===== Drive to other matchload bar and score same way as before - top goal ====== */

  wait(1, sec);

  stopIntake();

  stopTopRoller();

  // }

}

void hurrs_auton2() {
    default_constants();

  using namespace sdrive;

  toggleMatchLoader();

  spinIntake(1);

  s_drive(32, 90);

  wait(.25, sec);

  spinTopRoller(1);

  s_drive(-20);

  stopTopRoller();

  stopIntake();

  toggleMatchLoader();

  spinIntake(1);

  s_drive(20, 160);

  spinIntake(-1);

  s_drive(-10, 180);

  spinIntake(1);

  s_drive(50);

  s_drive(0, 200);

  toggleMiddleGoal();

  spinTopRoller(1);

  s_drive(-16);

  wait(.25, sec);

  stopTopRoller();

  toggleMatchLoader();

  toggleMiddleGoal();

  s_drive(32, 90);

  wait(.25, sec);

  spinTopRoller(1);

  s_drive(-20);

  wait(1, sec);

  stopIntake();

  stopTopRoller();

}



void s_drive_chain_test() {
  default_constants();

  using namespace sdrive;

  s_drive(32);
  
  s_drive(20, 90);
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
  chassis.turn_to_angle(-90);
  chassis.drive_distance(12);
  chassis.drive_distance(-12);
  chassis.right_swing_to_angle(90);
  chassis.drive_distance(22);
  // chassis.turn_to_angle(0);
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
  chassis.set_drive_constants(8, 0.8, 0.03, 0.8, 3);
  chassis.set_heading_constants(6, .4, 0, 11, 0);

  chassis.set_turn_constants(7, 0.30, 0.03, 1.56, 15);
  chassis.set_swing_constants(9, 0.30, 0.05, 2, 15);

  chassis.set_drive_exit_conditions(2, 200, 4000);
  chassis.set_turn_exit_conditions(1, 200, 1500);
  chassis.set_swing_exit_conditions(1, 200, 1500);
}

void test_functions (){
  default_constants();
  chassis.set_coordinates(0, 0, 0);

  using namespace sdrive;

  s_drive(62);

  // chassis.turn_to_angle(145);
}

void right_side_9 (){
  default_constants();
  chassis.set_coordinates(0, 0, 0);

}

void prog_skills (){
  default_constants();

  chassis.set_coordinates(0, 0, 0);

  chassis.drive_distance(34.5);

  chassis.turn_to_angle(90);

  //toggleMatchLoader();

  spinIntake(1);

  chassis.drive_max_voltage = 5;

  wait(.5, sec);

  chassis.drive_distance(13);

  wait(2.5, sec);

  chassis.drive_distance(-30);

  spinTopRoller(1);

  //toggleGuard();

  //wait(2.5, sec);

  stopTopRoller();

  chassis.drive_distance(6);

  // toggleMatchLoader;
  //toggleGuard;

  chassis.turn_to_angle(210);

  chassis.drive_max_voltage = 4;

  spinIntake(1);

  chassis.drive_distance(24);

  wait(.25, sec);

  chassis.drive_distance(4);

  wait(.25, sec);

  chassis.drive_distance(-26);

  chassis.turn_to_angle(90);

  // toggleGuard();

  chassis.drive_distance(-7);

  spinTopRoller(1);

  // wait(2.5, sec);

  chassis.drive_distance(14);
  
  chassis.turn_to_angle(175.8);

  chassis.drive_max_voltage = 7;
  chassis.drive_distance(69);

  chassis.turn_to_angle(180);

  // second half
  
  chassis.drive_distance(34.5);

  chassis.turn_to_angle(90);

  //toggleMatchLoader();

  spinIntake(1);

  chassis.drive_max_voltage = 5;

  wait(.5, sec);

  chassis.drive_distance(13);

  wait(2.5, sec);

  chassis.drive_distance(-30);

  spinTopRoller(1);

  //toggleGuard();

  //wait(2.5, sec);

  stopTopRoller();

  chassis.drive_distance(6);

  // toggleMatchLoader;
  //toggleGuard;

  chassis.turn_to_angle(30);

  chassis.drive_max_voltage = 4;

  spinIntake(1);

  chassis.drive_distance(24);

  wait(.25, sec);

  chassis.drive_distance(4);

  wait(.25, sec);

  chassis.drive_distance(-26);

  chassis.turn_to_angle(90);

  // toggleGuard();

  chassis.drive_distance(-7);

  spinTopRoller(1);

  // wait(2.5, sec);
}