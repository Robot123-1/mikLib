#include "vex.h"

using namespace vex;
using namespace mik;

void default_constants(void) {
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
    chassis.set_turn_constants(12, .275, 0.012, 5.805, 15);
    chassis.set_drive_constants(10, 1.427, 0, 8.000, .0);
    chassis.set_heading_constants(6, 0.921, 0.046, 7.767, 0.300);
    chassis.set_swing_constants(12, .437, .0295, 3.486, 15);

    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_turn_exit_conditions(1.5, 75, 2000);
    chassis.set_drive_exit_conditions(1, 75, 3000);
    chassis.set_swing_exit_conditions(1.25, 75, 3000);

}

void odom_constants(void) {
    default_constants();
    chassis.heading_max_voltage = 10;
    chassis.drive_max_voltage = 8;
    chassis.drive_settle_error = 3;
    chassis.boomerang_lead = .5;
    chassis.boomerang_setback = 2;    
}

std::string template_auto(bool calibrate, auto_variation var, bool get_name) {
    /* The first variation will be this auto */
    if (var == one) {}

    /* We declare and allow a second variation of this auto; 
    You may want this if you want a different movements in the same starting configuration */
    if (var == two) { return template_auto_other_variation(calibrate, get_name); }

    if (get_name) { /* Give a desciption of your auto */ return "template auto 1 (3 objs)"; }
    if (calibrate) {
        /* Initialize robots starting position "https://path.jerryio.com/" and/or add extra movements to line up robots 
        starting position **IF MOVING DURING CALIBRATION DO BEFORE FIELD CONTROLLER PLUG IN** */
        chassis.set_coordinates(55, 23.5, 90);
    
        /* Example of turning before auto is ran */
        chassis.turn_max_voltage = 6; 
        chassis.turn_to_angle(45);

        return "";
    }
    
    /* We now run the auto */ 
    chassis.drive_distance(24);
    task::sleep(500);
    chassis.drive_distance(-24);
    task::sleep(500);

    return "";
}
std::string template_auto_other_variation(bool calibrate, bool get_name) {
    if (get_name) { return "template auto 2 (4 objs)"; }
    
    // Mirror template_auto() from the x-axis
    chassis.mirror_all_auton_y_pos();
    
    if (calibrate) {
        // Coordinates will be set to (55, -23.5) as y_pos is mirrored
        template_auto(calibrate, one, get_name);
        return "";
    }
    
    // Run auto, make sure to pass in one as var.
    template_auto(calibrate, one, get_name);

    return "";
}


std::string blue_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    chassis.disable_mirroring();
    if (get_name) { return "blue left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(-46.62, 6.89, 269.99);
        return "";
    }
        odom_constants();
        chassis.turn_to_point(-21.798, 22.278, {.angle_offset = 180});
        assembly.spin_score_motors(fwd, 12);
        chassis.drive_to_point(-21.798, 22.278, {.max_voltage = 6});
        assembly.matchLoaderPiston.open();
        wait(1.2, sec);
        assembly.spin_score_motors(fwd, 10);
        assembly.matchLoaderPiston.close();
        chassis.turn_to_point(-7.628, 9.625);
        chassis.drive_to_point(-7.628, 9.625, {.max_voltage = 5});
        assembly.spin_score_motors(fwd, 10);
        assembly.set_goal_state(Assembly::GoalState::MidGoal);
        wait(1.3, sec);
        assembly.set_goal_state(Assembly::GoalState::BallLock);
        assembly.spin_score_motors(fwd, 8);
        chassis.turn_to_point(-47.221, 45.167, {.angle_offset = 180});
        chassis.drive_to_point(-47.221, 45.167, {.max_voltage = 9});
        chassis.turn_to_point(-61.835, 47.846, {.angle_offset = 180});
        assembly.matchLoaderPiston.open();
        chassis.drive_to_point(-61.835, 47.846, {.max_voltage = 7, .timeout = 965});
        chassis.turn_to_point(-24.717, 47.111);
        chassis.drive_to_point(-24.717, 47.111, {.max_voltage = 7, .timeout = 1250});
        assembly.spin_score_motors(fwd, 12);
        assembly.set_goal_state(Assembly::GoalState::LongGoal);
        wait(2, sec);
    return "";
}
std::string blue_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_left_elim(bool calibrate, auto_variation var, bool get_name) {   
    chassis.mirror_all_auton_y_pos();
    if (get_name) { return "blue left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    chassis.disable_mirroring();
    if (get_name) { return "blue right winpoint"; }
    if (calibrate) {
        
        chassis.set_coordinates(-46.62, -6.89, 269.99);

        return "";
    }
    odom_constants();
    chassis.turn_to_point(-21.798, -22.278, {.angle_offset = 180});
    assembly.spin_score_motors(fwd, 12);
    chassis.drive_to_point(-21.798, -22.278, {.max_voltage = 6});
    assembly.matchLoaderPiston.open();
    wait(.5, sec);
    assembly.spin_score_motors(fwd, 7);
    assembly.matchLoaderPiston.close();
    chassis.turn_to_point(-47.244, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.244, -45.167, {.max_voltage = 9});
    wait(.5, sec);
    chassis.turn_to_point(-24.728, -46.829);
    chassis.drive_to_point(-24.728, -46.829, {.max_voltage = 8, .timeout = 670});
    assembly.spin_score_motors(fwd, 12);
    assembly.set_goal_state(Assembly::GoalState::LongGoal);
    wait(2, sec);
    assembly.set_goal_state(Assembly::GoalState::BallLock);
    assembly.spin_score_motors(fwd, 8);
    chassis.turn_to_point(-47.244, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.244, -45.167, {.max_voltage = 7});
    assembly.matchLoaderPiston.open();
    chassis.turn_to_point(-61.162, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-61.162, -45.167, {.max_voltage = 7, .timeout = 790});
    chassis.turn_to_point(-24.728, -46.829);
    chassis.drive_to_point(-24.728, -46.829, {.max_voltage = 8, .timeout = 990});
    assembly.spin_score_motors(fwd, 12);
    assembly.set_goal_state(Assembly::GoalState::LongGoal);
    wait(4, sec);
    
    return "";
}
std::string blue_right_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right elim"; }
    if (calibrate) {
        odom_constants();
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}

std::string red_left_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    chassis.disable_mirroring();
    if (get_name) { return "red left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(-46.62, 6.89, 269.99);

        return "";
    }
        odom_constants();
        chassis.turn_to_point(-21.798, 22.278, {.angle_offset = 180});
        assembly.spin_score_motors(fwd, 12);
        chassis.drive_to_point(-21.798, 22.278, {.max_voltage = 6});
        assembly.matchLoaderPiston.open();
        wait(1.2, sec);
        assembly.spin_score_motors(fwd, 10);
        assembly.matchLoaderPiston.close();
        chassis.turn_to_point(-7.628, 9.625);
        chassis.drive_to_point(-7.628, 9.625, {.max_voltage = 5});
        assembly.spin_score_motors(fwd, 10);
        assembly.set_goal_state(Assembly::GoalState::MidGoal);
        wait(1.3, sec);
        assembly.set_goal_state(Assembly::GoalState::BallLock);
        assembly.spin_score_motors(fwd, 8);
        chassis.turn_to_point(-47.221, 45.167, {.angle_offset = 180});
        chassis.drive_to_point(-47.221, 45.167, {.max_voltage = 9});
        chassis.turn_to_point(-61.835, 47.846, {.angle_offset = 180});
        assembly.matchLoaderPiston.open();
        chassis.drive_to_point(-61.835, 47.846, {.max_voltage = 7, .timeout = 965});
        chassis.turn_to_point(-24.717, 47.111);
        chassis.drive_to_point(-24.717, 47.111, {.max_voltage = 7, .timeout = 1250});
        assembly.spin_score_motors(fwd, 12);
        assembly.set_goal_state(Assembly::GoalState::LongGoal);
        wait(2, sec);
    return "";
}
std::string red_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string red_left_elim(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
    
    return "";
}
std::string red_right_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    chassis.disable_mirroring();
    if (get_name) { return "red right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(-46.62, -6.89, 269.99);
    return "";
}
    odom_constants();
    chassis.turn_to_point(-21.798, -22.278, {.angle_offset = 180});
    assembly.spin_score_motors(fwd, 12);
    chassis.drive_to_point(-21.798, -22.278, {.max_voltage = 6});
    assembly.matchLoaderPiston.open();
    wait(.5, sec);
    assembly.spin_score_motors(fwd, 7);
    assembly.matchLoaderPiston.close();
    chassis.turn_to_point(-47.244, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.244, -45.167, {.max_voltage = 9});
    wait(.5, sec);
    chassis.turn_to_point(-24.728, -46.829);
    chassis.drive_to_point(-24.728, -46.829, {.max_voltage = 8, .timeout = 670});
    assembly.spin_score_motors(fwd, 12);
    assembly.set_goal_state(Assembly::GoalState::LongGoal);
    wait(2, sec);
    assembly.set_goal_state(Assembly::GoalState::BallLock);
    assembly.spin_score_motors(fwd, 8);
    chassis.turn_to_point(-47.244, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.244, -45.167, {.max_voltage = 7});
    assembly.matchLoaderPiston.open();
    chassis.turn_to_point(-61.162, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-61.162, -45.167, {.max_voltage = 7, .timeout = 790});
    chassis.turn_to_point(-24.728, -46.829);
    chassis.drive_to_point(-24.728, -46.829, {.max_voltage = 8, .timeout = 990});
    assembly.spin_score_motors(fwd, 12);
    assembly.set_goal_state(Assembly::GoalState::LongGoal);
    wait(4, sec);
    return "";
}


std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "red right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
    odom_constants();
    chassis.turn_to_point(0, 60);
    chassis.drive_to_point(0, 60);


    return "";
}
std::string red_right_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "red right elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}

std::string skills(bool calibrate, auto_variation var, bool get_name) {
    chassis.disable_mirroring();
    if (get_name) { return "skills"; }
    if (calibrate) {
        odom_constants();
        chassis.set_coordinates(-46.62, -6.89, 269.99);
        return "";
    }
    assembly.spin_score_motors(fwd, 8);
    chassis.turn_to_point(-47.244, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.244, -45.167, {.max_voltage = 7});
    assembly.matchLoaderPiston.open();
    chassis.turn_to_point(-61.162, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-61.162, -45.167, {.max_voltage = 7, .timeout = 1700});
    chassis.turn_to_point(-24.728, -46.829);
    chassis.drive_to_point(-24.728, -46.829, {.max_voltage = 8, .timeout = 1500});
    assembly.matchLoaderPiston.close();
    assembly.spin_score_motors(fwd, 12);
    assembly.set_goal_state(Assembly::GoalState::LongGoal);
    wait(3, sec);
    assembly.set_goal_state(Assembly::GoalState::BallLock);
    chassis.turn_to_point(-47.244, -45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.244, -45.167, {.max_voltage = 7});
    chassis.turn_to_point(-21.798, -22.278, {.angle_offset = 180});
    assembly.spin_score_motors(fwd, 12);
    chassis.drive_to_point(-21.798, -22.278, {.max_voltage = 6});
    assembly.matchLoaderPiston.open();
    wait(.7, sec);
    assembly.spin_score_motors(fwd, 7);
    assembly.matchLoaderPiston.close();
    chassis.turn_to_point(-21.798, 22.278, {.angle_offset = 180});
    chassis.drive_to_point(-21.798, 22.278, {.max_voltage = 4});
    assembly.matchLoaderPiston.open();
    wait(1.2, sec);
    chassis.turn_to_point(-7.628, 9.625);
    chassis.drive_to_point(-7.628, 9.625, {.max_voltage = 5});
    assembly.spin_score_motors(fwd, 8);
    assembly.set_goal_state(Assembly::GoalState::MidGoal);
    wait(3.3, sec);
    assembly.set_goal_state(Assembly::GoalState::BallLock);
    chassis.turn_to_point(-47.221, 45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.221, 45.167, {.max_voltage = 8});
    chassis.turn_to_point(-61.835, 47.846, {.angle_offset = 180});
    chassis.drive_to_point(-61.835, 47.846, {.max_voltage = 7, .timeout = 3000});
    chassis.turn_to_point(-24.717, 47.111);
    chassis.drive_to_point(-24.717, 47.111, {.max_voltage = 7, .timeout = 3000});
    assembly.spin_score_motors(fwd, 12);
    assembly.set_goal_state(Assembly::GoalState::LongGoal);
    wait(4, sec);
    assembly.matchLoaderPiston.close();
    chassis.turn_to_point(-47.221, 45.167, {.angle_offset = 180});
    chassis.drive_to_point(-47.221, 45.167, {.max_voltage = 7});
    chassis.turn_to_point(-44.866, -1.272, {.angle_offset = 180});
    chassis.drive_to_point(-44.866, -1.272, {.max_voltage = 7});
    chassis.turn_to_point(-66.815, -0.824, {.angle_offset = 180});
    assembly.armPiston.open();
    chassis.left_drive.spin(reverse, 6, volt);
    chassis.right_drive.spin(reverse, 6, volt);
    wait(2.5, sec);
    chassis.left_drive.stop(hold);
    chassis.right_drive.stop(hold);

    return "";
}