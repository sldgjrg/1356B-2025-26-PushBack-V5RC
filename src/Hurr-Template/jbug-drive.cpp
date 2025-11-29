// #include "Hurr-Template/jbug-drive.h"
// #include "JAR-Template/drive.h"
// #include "vex.h"
// #include <cmath>
// #include <iostream>

// using namespace vex;

// void resetEncoders(){
//   LeftDrive.resetPosition();
//   RightDrive.resetPosition();
// }

// double wheelRevToInches(double rev) {
//   return rev * 3.14159265358979323846 * 3.25 * 0.75;
// }

// double avgDistanceIn() {
//   double L = LeftFront.position(rev) + LeftMiddle.position(rev) + LeftBack.position(rev);
//   double R = RightFront.position(rev) + RightMiddle.position(rev) + RightBack.position(rev);
//   return wheelRevToInches((L + R) / 6);
// }

// void jbugdrive(float targetDist, float maxVelocity) {
//     resetEncoders();

//     float driveOutput;
//     float currentPosition;
//     float k = pow(targetDist, 1.0/2.0); 
//     float MIN_VOLTAGE = 1.8;

//     while(true) {
//         currentPosition = avgDistanceIn();

//         if(currentPosition < targetDist * 0.5) {
//             driveOutput = maxVelocity / (1 + exp(-k * (currentPosition - 0.02 * targetDist))); 
//             driveOutput = std::max(driveOutput, MIN_VOLTAGE);
//             chassis.drive_with_voltage(driveOutput, driveOutput);
//         } 
//         else {
//             driveOutput = maxVelocity / (1 + exp(0.4 * (currentPosition - 0.75 * targetDist)));
//             chassis.drive_with_voltage(driveOutput, driveOutput);
//         } 

//         task::sleep(10);

//         if(currentPosition >= targetDist) {
//             break;
//         }
//     }
//     LeftDrive.stop(hold);
//     RightDrive.stop(hold);
// }

// desired_heading = heading;
//   PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
//   PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
//   float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
//   float average_position = start_average_position;
//   while(drivePID.is_settled() == false){
//     average_position = (get_left_position_in()+get_right_position_in())/2.0;
//     float drive_error = distance+start_average_position-average_position;
//     float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
//     float drive_output = drivePID.compute(drive_error);
//     float heading_output = headingPID.compute(heading_error);

//     drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
//     heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

//     drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
//     task::sleep(10);