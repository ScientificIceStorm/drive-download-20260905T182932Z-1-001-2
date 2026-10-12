#include "main.h"
#include "auton.hpp"
#include "auton_selector.hpp"
#include "robot_config.hpp"
#include <cmath>

extern pros::adi::DigitalOut claw;
extern pros::adi::DigitalOut pneumatic_h;
extern pros::MotorGroup lift_mg;

namespace robot {
pros::MotorGroup left({9, 20, -19});
pros::MotorGroup right({10, 18, -8});
pros::Imu imu(11);
pros::Rotation tracking_sensor(robot_config::tracking_sensor_port);

lemlib::Chassis& chassis() {
    static lemlib::Drivetrain drive(&left, &right, robot_config::track_width,
                                   robot_config::drive_wheel_diameter, robot_config::drive_wheel_rpm, 2);
    static lemlib::TrackingWheel tracking_wheel(&tracking_sensor, robot_config::tracking_wheel_diameter,
                                              robot_config::tracking_wheel_offset, robot_config::tracking_wheel_ratio);
    static lemlib::OdomSensors sensors(&tracking_wheel, nullptr, nullptr, nullptr, &imu);
    static lemlib::ControllerSettings linear(10, 0, 3, 3, 1, 100, 3, 500, 20);
    static lemlib::ControllerSettings angular(2, 0, 10, 3, 1, 100, 3, 500, 0);
    static lemlib::Chassis instance(drive, linear, angular, sensors);
    return instance;
}

void initialize() {
    left.set_gearing(pros::E_MOTOR_GEARSET_06, 0);
    left.set_gearing(pros::E_MOTOR_GEARSET_06, 1);
    right.set_gearing(pros::E_MOTOR_GEARSET_06, 0);
    right.set_gearing(pros::E_MOTOR_GEARSET_06, 1);
    left.set_gearing(pros::E_MOTOR_GEARSET_18, 2);
    right.set_gearing(pros::E_MOTOR_GEARSET_18, 2);
    imu.reset(true);
    chassis().calibrate(false);
    pros::delay(30);
    chassis().setPose(0, 0, 0);
}

void stop() {
    chassis().cancelAllMotions();
    left.move(0);
    right.move(0);
    lift_mg.brake();
}

void run_auton() {
    stop();
    run_selected_auton();
    chassis().waitUntilDone();
    stop();
}
}

void start_at(float x, float y, float heading) {
    robot::chassis().setPose(x, y, heading);
}

void move_to(float x, float y, int speed, bool backwards, int timeout_ms, bool async) {
    robot::chassis().moveToPoint(x, y, timeout_ms,
                               {.forwards = !backwards, .maxSpeed = static_cast<float>(speed)}, async);
}

void open_claw() {
    claw.set_value(false);
    pros::delay(300);
}

void raise_piston() {
    pneumatic_h.set_value(true);
}

void lower_piston() {
    pneumatic_h.set_value(false);
}

void close_claw() {
    claw.set_value(true);
    pros::delay(300);
}

void lift_to(float degrees, int timeout_ms) {
    lift_mg.move_absolute(degrees, 60);
    int settled_ms = 0;
    for (int elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
        const auto positions = lift_mg.get_position_all();
        const bool at_height = std::fabs(positions[0] - degrees) <= 10 &&
                               std::fabs(positions[1] - degrees) <= 10;
        settled_ms = at_height ? settled_ms + 10 : 0;
        if (settled_ms >= 100) break;
        pros::delay(10);
    }
    lift_mg.brake();
}
