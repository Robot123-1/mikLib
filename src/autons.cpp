#include "vex.h"

using namespace vex;
using namespace mik;

void default_constants(void) {
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
    chassis.set_turn_constants(12, .275, 0.012, 5.805, 15);
    chassis.set_drive_constants(10, 1.427, 0, 8.000, .0);
    chassis.set_heading_constants(6, 0.921, 0.026, 7.648, 0);
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
    if (get_name) { return "blue left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

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
    if (get_name) { return "blue left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

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
    if (get_name) { return "red left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    
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
    if (get_name) { return "red right winpoint"; }
    if (calibrate) {
        odom_constants();
        chassis.set_coordinates(-39.221, 0, 270);
        assembly.middleGoalPiston.close();
    return "";
}
    assembly.middleGoalPiston.close();
    chassis.turn_to_point(-24.708,-21.877, {.angle_offset = 180});
    assembly.intakeMotor.spin(fwd, 12, volt);
    chassis.drive_to_point(-24.708,-21.877, {.max_voltage = 3});
    assembly.intakeMotor.stop();
    chassis.turn_to_point(-47.428, -47.128, {.angle_offset = 180});
    chassis.drive_to_point(-47.428, -47.128, {.max_voltage = 4});
    chassis.turn_to_point(-18.437, -46.514);
    chassis.drive_to_point(-18.437, -46.514, {.max_voltage = 5, .timeout = 2000});
    assembly.intakeMotor.spin(fwd, 12, volt);
    assembly.scoringMotor.spin(reverse, 12, volt);
    wait (3.3, sec);
    assembly.scoringMotor.stop();
    chassis.turn_to_point(-72.095, -46.917, {.angle_offset = 180});
    assembly.matchLoaderPiston.close();
    chassis.drive_to_point(-72.095, -46.917, {.max_voltage = 8.8, .timeout = 1500});
    wait (0.5, sec);
    chassis.turn_to_point(-17.989, -45.618);
    chassis.drive_to_point(-17.989, -45.618, {.max_voltage = 4, .timeout = 5000});
    assembly.scoringMotor.spin(reverse, 12, volt);
    



   





    return "";
}


std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "red right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }

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
    if (get_name) { return "skills"; }
    if (calibrate) {
        odom_constants();
        chassis.set_coordinates(-39.266, 0, 90);
        return "";
    }
    assembly.middleGoalPiston.close();
    chassis.turn_to_point(-22.917, 23.812, {.angle_offset = 180});
    assembly.intakeMotor.spin(fwd, 12, volt);
    chassis.drive_to_point(-22.917, 23.812, {.max_voltage = 3});
    chassis.turn_to_point(-46.82, 46.698, {.angle_offset = 180});
    chassis.drive_to_point(-46.82, 46.698);
    chassis.turn_to_point(-24.727, 46.997);
    chassis.drive_to_point(-24.727, 46.997, {.max_voltage = 6, .timeout = 3000});
    assembly.scoringMotor.spin(reverse, 12, volt);
    wait (3,sec);
    chassis.turn_to_point(-46.82, 46.698, {.angle_offset = 180});
    chassis.drive_to_point(-46.82, 46.698, {.max_voltage = 8});
    chassis.turn_to_point(-73.54, 46.698, {.angle_offset = 180});
    assembly.intakeMotor.spin(fwd, 12, volt);
    assembly.matchLoaderPiston.close();
    wait (1, sec);
    chassis.drive_to_point(-73.54, 46.698, {.max_voltage = 8, .timeout = 3000});
    chassis.drive_to_point(-57.184, 46.657, {.max_voltage = 1.2});
    wait (4, sec);
    //match loading
    chassis.turn_to_point(-46.82, 46.698);
    chassis.drive_to_point(-46.82, 46.698, {.max_voltage = 3});
    chassis.turn_to_point(-24.727, 46.997);
    chassis.drive_to_point(-24.727, 46.997, {.max_voltage = 8, .timeout = 5000});
    assembly.scoringMotor.spin(reverse, 12, volt);
    assembly.matchLoaderPiston.open();
    wait (5, sec);
    //scored in long goal
    chassis.turn_to_point(-54.183, 46.774, {.angle_offset = 180});
    chassis.drive_to_point(-54.183, 46.774, {.max_voltage = 7});
    chassis.turn_to_point(-60.392, 19.872);
    chassis.drive_to_point(-60.392, 19.872, {.max_voltage = 7});
    //lift odom pods
    //need to use motor encoders from here
    return "";
}