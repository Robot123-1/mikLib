#include "vex.h"

using namespace vex;

Assembly::Assembly(
    mik::motor intakeMotor,
    mik::motor scoringMotor,
    mik::piston middleGoalPiston,
    mik::piston armPiston,
    mik::piston matchLoaderPiston,
    mik::piston odomPodLifter
) :
    intakeMotor(intakeMotor),
    scoringMotor(scoringMotor),
    middleGoalPiston(middleGoalPiston),
    armPiston(armPiston),
    matchLoaderPiston(matchLoaderPiston),
    odomPodLifter(odomPodLifter)
{
}

void Assembly::init() {
    // Optional one-time setup
}

void Assembly::control() {
    intake_motor_control();
    scoring_motor_control();
    middle_goal_piston_control();
    arm_piston_control();
    match_loader_piston_control();
    odom_pod_lifter_control();
}

void Assembly::intake_motor_control() {
    if (Controller.ButtonL1.pressing() || Controller.ButtonR1.pressing() || Controller.ButtonL2.pressing()) {
        intakeMotor.spin(fwd, 12, volt);   // L1, R1 & L2 = forward
    }
    else if (Controller.ButtonR2.pressing()) {
        intakeMotor.spin(reverse, 12, volt); // R2 = reverse
    }
    else {
        intakeMotor.stop(brakeType::coast);
    }
}



void Assembly::scoring_motor_control() {
    if (Controller.ButtonL1.pressing()) {
        scoringMotor.spin(reverse, 12, volt);
    }
    else {
        scoringMotor.stop(brakeType::coast);
    }
}



void Assembly::middle_goal_piston_control() {
    static bool lastA = false;
    bool nowA = Controller.ButtonA.pressing();
    if (nowA && !lastA) {
        middleGoalPiston.toggle();
    }
    lastA = nowA;
}

void Assembly::arm_piston_control() {
    static bool lastY = false;
    bool nowY = Controller.ButtonY.pressing();
    if (nowY && !lastY) {
        armPiston.toggle();
    }
    lastY = nowY;
}

void Assembly::match_loader_piston_control() {
    if (Controller.ButtonL2.pressing()) {
        matchLoaderPiston.close();
    }
    else {
        matchLoaderPiston.open();
    }
}

void Assembly::odom_pod_lifter_control() {
    static bool lastRight = false;
    bool nowRight = Controller.ButtonRight.pressing();
    if (nowRight && !lastRight) {
        odomPodLifter.toggle();
    }
    lastRight = nowRight;
}

void Assembly::intake_motors_control() {
    // Keep this only if your header still declares it.
    // For a single intake motor, just call the single-motor function:
    intake_motor_control();
}
