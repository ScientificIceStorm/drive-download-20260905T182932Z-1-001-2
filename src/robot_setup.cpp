#include "main.h"
#include "auton.hpp"
#include "auton_selector.hpp"
#include "robot_config.hpp"
#include <cmath>
#include <mutex>

extern pros::adi::DigitalOut claw;
extern pros::adi::DigitalOut pneumatic_h;
extern pros::MotorGroup lift_mg;
extern pros::Motor auxiliary_motor;
extern pros::Motor auxiliary_motor_6;

namespace {
pros::Mutex lift_mutex;
unsigned int lift_command = 0;
}

namespace robot {
pros::MotorGroup left({-18, -19, -17});
pros::MotorGroup right({20, 11, 3});
pros::Imu imu(8);
pros::Rotation tracking_sensor(-13);
pros::Distance cup_sensor(16);
pros::Distance wall_sensor(7);
pros::Distance right_wall_sensor(12);

lemlib::Chassis& chassis() {
    static lemlib::Drivetrain drive(&left, &right, robot_config::track_width,
                                   robot_config::drive_wheel_diameter, robot_config::drive_wheel_rpm, 2);
    static lemlib::TrackingWheel tracking_wheel(&tracking_sensor, lemlib::Omniwheel::NEW_2, -0.5);
    static lemlib::OdomSensors sensors(&tracking_wheel, nullptr, nullptr, nullptr, &imu);
    static lemlib::ControllerSettings linear(10, 0, 3, 3, 0.25, 100, 0.5, 500, 20);
    static lemlib::ControllerSettings angular(2.7, 0, 10, 3, 0.5, 100, 1, 500, 0);
    static lemlib::Chassis instance(drive, linear, angular, sensors);
    return instance;
}

void initialize() {
    for (auto* group : {&left, &right}) {
        for (int i = 0; i < group->size(); ++i) {
            const auto type = group->get_type(i);
            if (type == pros::MotorType::exp) group->set_gearing(pros::MotorGears::green, i);
            else if (type == pros::MotorType::v5) group->set_gearing(pros::MotorGears::blue, i);
        }
    }
    chassis().calibrate();
    pros::delay(100);
    chassis().setPose(0, 0, 0);
}

void stop() {
    std::lock_guard<pros::Mutex> lock(lift_mutex);
    ++lift_command;
    auxiliary_motor_6.move(0);
    auxiliary_motor.move(0);
    chassis().cancelAllMotions();
    left.move(0);
    right.move(0);
    lift_mg.brake();
}

void run_auton() {
    lift_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    stop();
    run_selected_auton();
    chassis().waitUntilDone();
    stop();
    pros::delay(5000);
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
    claw.set_value(true);
    pros::delay(300);
}

void raise_piston() {
    pneumatic_h.set_value(true);
}

void lower_piston() {
    pneumatic_h.set_value(false);
}

void close_claw() {
    claw.set_value(false);
    pros::delay(300);
}

void lift_to(float degrees, int timeout_ms, bool async) {
    unsigned int command;
    {
        std::lock_guard<pros::Mutex> lock(lift_mutex);
        command = ++lift_command;
        lift_mg.move_absolute(degrees, 60);
    }
    auto wait_for_lift = [degrees, timeout_ms, command] {
        int settled_ms = 0;
        for (int elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
            {
                std::lock_guard<pros::Mutex> lock(lift_mutex);
                if (command != lift_command) return;
                const auto positions = lift_mg.get_position_all();
                const bool at_height = std::fabs(positions[0] - degrees) <= 10 &&
                                       std::fabs(positions[1] - degrees) <= 10;
                settled_ms = at_height ? settled_ms + 10 : 0;
                if (settled_ms >= 100) break;
            }
            pros::delay(10);
        }
        std::lock_guard<pros::Mutex> lock(lift_mutex);
        if (command == lift_command) lift_mg.brake();
    };
    if (async) {
        pros::Task task(wait_for_lift);
    } else {
        wait_for_lift();
    }
}

void spin_motor(float degrees, int rpm, int timeout_ms) {
    const double target = auxiliary_motor.get_position() + degrees;
    auxiliary_motor.move_relative(degrees, rpm);
    for (int elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
        if (std::fabs(auxiliary_motor.get_position() - target) <= 5) break;
        pros::delay(10);
    }
    auxiliary_motor.move(0);
}
