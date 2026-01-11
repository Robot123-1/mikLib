#include "vex.h"

using namespace vex;

Assembly::Assembly(
    mik::motor intakeMotor,
    mik::motor scoringMotor,
    mik::piston middleGoalPiston,
    mik::piston armPiston
) :
    intakeMotor(intakeMotor),
    scoringMotor(scoringMotor),
    middleGoalPiston(middleGoalPiston),
    armPiston(armPiston)
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
}

void Assembly::intake_motor_control() {
    if (Controller.ButtonL1.pressing() || Controller.ButtonR1.pressing()) {
        intakeMotor.spin(fwd, 12, volt);   // L1 & R1 = forward
    }
    else if (Controller.ButtonL2.pressing() || Controller.ButtonR2.pressing()) {
        intakeMotor.spin(reverse, 12, volt); // L2 & R2 = reverse
    }
    else {
        intakeMotor.stop(brakeType::coast);
    }
}



void Assembly::scoring_motor_control() {
    // l1 / l2 (unchanged)
    if (Controller.ButtonL1.pressing()) {
        scoringMotor.spin(reverse, 12, volt);
    }
    else if (Controller.ButtonL2.pressing()) {
        scoringMotor.spin(fwd, 12, volt);
    }

    // L1 / L2
    else if (Controller.ButtonL1.pressing()) {
        scoringMotor.spin(fwd, 12, volt);     // L1 (unchanged)
    }
    else if (Controller.ButtonL2.pressing()) {
        scoringMotor.spin(reverse, 12, volt); // L2 FIXED (reversed)
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
    static bool lastB = false;
    bool nowB = Controller.ButtonB.pressing();
    if (nowB && !lastB) {
        armPiston.toggle();
    }
    lastB = nowB;
}

void Assembly::intake_motors_control() {
    // Keep this only if your header still declares it.
    // For a single intake motor, just call the single-motor function:
    intake_motor_control();
}
