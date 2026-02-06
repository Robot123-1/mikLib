#include "vex.h"

using namespace vex;

Assembly::Assembly(
    mik::motor scoreMotor1,
    mik::motor scoreMotor2,
    mik::piston armPiston,
    mik::piston goalPiston1,
    mik::piston goalPiston2,
    mik::piston matchLoaderPiston
) :
    scoreMotor1(scoreMotor1),
    scoreMotor2(scoreMotor2),
    armPiston(armPiston),
    goalPiston1(goalPiston1),
    goalPiston2(goalPiston2),
    matchLoaderPiston(matchLoaderPiston)
{
}

void Assembly::init() {
    set_goal_state(GoalState::LongGoal);
}

void Assembly::control() {
    score_motor_control();
    goal_state_control();
    arm_piston_control();
    match_loader_piston_control();
}

void Assembly::score_motor_control() {
    const int DOUBLE_PRESS_THRESHOLD = 300; // milliseconds

    bool nowR1 = Controller.ButtonR1.pressing();
    bool nowR2 = Controller.ButtonR2.pressing();
    bool nowL1 = Controller.ButtonL1.pressing();
    bool nowL2 = Controller.ButtonL2.pressing();
    // R2 double-press-and-hold reverse
    if (nowR2 && !lastR2_) {
        int timeSinceLastPress = r2_press_timer_.time();
        r2_reverse_mode_ = (timeSinceLastPress < DOUBLE_PRESS_THRESHOLD);
        r2_press_timer_.reset();
    }

    if (nowR2) {
        spin_score_motors(r2_reverse_mode_ ? reverse : fwd, 9);
    }
    else if (nowR1 || nowL1 || nowL2) {
        spin_score_motors(fwd, 12);
    }
    else {
        stop_score_motors(brakeType::coast);
        r2_reverse_mode_ = false;
    }

    lastR2_ = nowR2;
}



void Assembly::goal_state_control() {
    if (Controller.ButtonL1.pressing()) {
        set_goal_state(GoalState::LongGoal);
    }
    else if (Controller.ButtonL2.pressing()) {
        set_goal_state(GoalState::MidGoal);
    }
    else {
        set_goal_state(GoalState::BallLock);
    }
}

void Assembly::set_goal_state(GoalState state) {
    goal_state_ = state;

    switch (goal_state_) {
        case GoalState::LongGoal:
            // both closed
            goalPiston1.close();
            goalPiston2.close();
            break;
        case GoalState::BallLock:
            // one up, one down (goal1 up, goal2 down)
            goalPiston1.open();
            goalPiston2.close();
            break;
        case GoalState::MidGoal:
        default:
            // both up
            goalPiston1.open();
            goalPiston2.open();
            break;
    }
}

void Assembly::match_loader_piston_control() {
    if (Controller.ButtonR1.pressing()) {
        matchLoaderPiston.open();
    }
    else {
        matchLoaderPiston.close();
    }
}

void Assembly::arm_piston_control() {
    static bool lastY = false;
    bool nowY = Controller.ButtonY.pressing();
    if (nowY && !lastY) {
        armPiston.toggle();
    }
    lastY = nowY;
}

void Assembly::spin_score_motors(directionType dir, double voltage) {
    scoreMotor1.spin(dir, voltage, volt);
    scoreMotor2.spin(dir, voltage, volt);
}

void Assembly::stop_score_motors(brakeType brake) {
    scoreMotor1.stop(brake);
    scoreMotor2.stop(brake);
}
