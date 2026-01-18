#pragma once

#include "vex.h"

using namespace vex;

class Assembly {
public:
    Assembly(
        mik::motor intakeMotor,
        mik::motor scoringMotor,
        mik::piston middleGoalPiston,
        mik::piston armPiston,
        mik::piston matchLoaderPiston,
        mik::piston odomPodLifter
    );

    void init();
    void control();

    void intake_motor_control();
    void scoring_motor_control();
    void intake_motors_control();
    void middle_goal_piston_control();
    void arm_piston_control();
    void match_loader_piston_control();
    void odom_pod_lifter_control();

    // Hardware
    mik::motor intakeMotor;
    mik::motor scoringMotor;
    mik::piston middleGoalPiston;
    mik::piston armPiston;
    mik::piston matchLoaderPiston;
    mik::piston odomPodLifter;
};