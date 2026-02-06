#pragma once

#include "vex.h"

using namespace vex;

class Assembly {
public:
    enum class GoalState {
        LongGoal,
        BallLock,
        MidGoal
    };

    Assembly(
        mik::motor scoreMotor1,
        mik::motor scoreMotor2,
        mik::piston armPiston,
        mik::piston goalPiston1,
        mik::piston goalPiston2,
        mik::piston matchLoaderPiston
    );

    void init();
    void control();

    void score_motor_control();
    void goal_state_control();
    void set_goal_state(GoalState state);
    void spin_score_motors(vex::directionType dir, double voltage);
    void stop_score_motors(vex::brakeType brake = vex::brakeType::coast);
    void arm_piston_control();
    void match_loader_piston_control();

    // Hardware
    mik::motor scoreMotor1;
    mik::motor scoreMotor2;
    mik::piston armPiston;
    mik::piston goalPiston1;
    mik::piston goalPiston2;
    mik::piston matchLoaderPiston;

private:
    GoalState goal_state_ = GoalState::LongGoal;
    vex::timer r2_press_timer_;
    bool lastR2_ = false;
    bool r2_reverse_mode_ = false;
};