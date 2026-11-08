#pragma once
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "robot_config.hpp"

void auton_steps();
void start_at(float x, float y, float heading);
void move_to(float x, float y, int speed = 40, bool backwards = false,
             int timeout_ms = robot_config::movement_timeout_ms, bool async = false);
void open_claw();
void close_claw();
void spin_motor(float degrees, int rpm = 100, int timeout_ms = robot_config::mechanism_timeout_ms);
void raise_piston();
void lower_piston();
void lift_to(float degrees, int timeout_ms = robot_config::mechanism_timeout_ms, bool async = false);

namespace robot {
extern pros::MotorGroup left;
extern pros::MotorGroup right;
extern pros::Imu imu;
extern pros::Distance cup_sensor;
extern pros::Distance wall_sensor;
extern pros::Distance right_wall_sensor;
lemlib::Chassis& chassis();
void initialize();
void run_auton();
void stop();
}
