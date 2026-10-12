#include "main.h"
#include "auton.hpp"
#include "auton_selector.hpp"
#include "robot_config.hpp"
#include <cmath>

pros::adi::DigitalOut claw('A', false);
pros::adi::DigitalOut pneumatic_h('H', false);
pros::MotorGroup lift_mg({5, -7});

void initialize() {
    lift_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    lift_mg.tare_position_all();
    lift_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    robot::initialize();
    initialize_selector();
}
void disabled() { robot::stop(); }
void competition_initialize() {}
void autonomous() { robot::run_auton(); }

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        const bool screen_run = take_run_request();
        if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) || screen_run) &&
            !pros::competition::is_connected()) {
            lift_mg.move(0);
            robot::run_auton();
        }
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        if (std::abs(forward) < 8) forward = 0;
        if (std::abs(turn) < 8) turn = 0;
        robot::chassis().arcade(forward, turn, true);
        const int lift_direction = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) -
                                   master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        if (lift_direction == 0) lift_mg.brake();
        else lift_mg.move(lift_direction * robot_config::lift_power);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
            claw.set_value(false);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            claw.set_value(true);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
            pneumatic_h.set_value(true);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
            pneumatic_h.set_value(false);
        pros::delay(20);
    }
}
