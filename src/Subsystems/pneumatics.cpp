#include "vex.h"


//opcontrol functions
bool matchloaderExtended = false;

bool guardExtended = false;

bool middlegoalExtended = false;

void toggleMatchLoader() {
    matchloader.set(!matchloaderExtended);
}

void toggleGuard() {
    guard.set(!guardExtended);
}

void toggleMiddleGoal() {
    middlegoal.set(!middlegoalExtended);
}

void matchloaderTask() {
    while (true) {
        if (master.ButtonRight.pressing()) {
            if (matchloaderExtended) {
                matchloader.set(false);
                wait(200, msec);
                matchloaderExtended = false;
            }
            else {
                matchloader.set(true);
                wait(200, msec);
                matchloaderExtended = true;
            }
        }
        wait(20, msec);
    }
}



void guardTask() {
    while (true) {
        if (master.ButtonA.pressing()) {
            if (guardExtended) {
                guard.set(false);
                wait(200, msec);
                guardExtended = false;
            }
            else {
                guard.set(true);
                wait(200, msec);
                guardExtended = true;
            }
        }
        wait(20, msec);
    }
}

void middlegoalTask() {
    while (true) {
        if (master.ButtonB.pressing()) {
            if (middlegoalExtended) {
                middlegoal.set(false);
                wait(200, msec);
                middlegoalExtended = false;
            }
            else {
                middlegoal.set(true);
                wait(200, msec);
                middlegoalExtended = true;
            }
        }
        wait(20, msec); // prevent jamming with debounce
    }
}