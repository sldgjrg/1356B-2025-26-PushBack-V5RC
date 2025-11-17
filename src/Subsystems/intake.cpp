#include "vex.h"

//auton functions
void spinIntake(int direction) {
    intake.spin(fwd, 12.7 * direction, volt);
}

void stopIntake() {
    intake.spin(fwd, 1, volt);
}

void spinTopRoller(int direction) {
    top_roller.spin(fwd, 12.7 * direction, volt);
}

void stopTopRoller() {
    top_roller.spin(fwd, 1, volt);
}

//opcontrol functions
void intakeTask() {
    while (true) {
        if (master.ButtonR1.pressing()) {
            intake.spin(fwd, 12.7, volt);
        } 
        else if (master.ButtonR2.pressing()) {
            intake.spin(reverse, 12.7, volt);
        }
        else if (master.ButtonL1.pressing()) {
            intake.spin(fwd, 12.7, volt);
            top_roller.spin(fwd, 12.7, volt);
        } 
        else if (master.ButtonL2.pressing()) {
            intake.spin(reverse, 12.7, volt);
            top_roller.spin(reverse, 12.7, volt);
        }
        else {
            top_roller.spin(fwd, 0, volt);
            intake.spin(fwd, 1, volt);
        }
        wait(20, msec);
    }
}