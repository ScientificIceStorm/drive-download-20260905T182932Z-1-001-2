#pragma once
#include "lemlib/api.hpp"

void auton_steps();
void start_at(float x, float y, float heading);
void move_to(float x, float y, int speed = 40, bool backwards = false, int timeout_ms = 2000, bool async = false);
void open_claw();
void close_claw();
void raise_piston();
void lower_piston();
void lift_to(float degrees, int timeout_ms = 1500);

namespace robot {
extern pros::MotorGroup left;
extern pros::MotorGroup right;
extern pros::Imu imu;
lemlib::Chassis& chassis();
void initialize();
void run_auton();
void stop();
}
