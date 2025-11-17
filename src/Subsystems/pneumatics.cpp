#include "vex.h"


//opcontrol functions
bool matchloaderExtended = false;

bool guardExtended = false;

void toggleMatchLoader() {
    matchloader.set(!matchloaderExtended);
}

void toggleGuard() {
    guard.set(!guardExtended);
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