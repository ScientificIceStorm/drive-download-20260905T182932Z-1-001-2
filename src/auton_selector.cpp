#include "main.h"
#include "auton.hpp"
#include "auton_selector.hpp"
#include <atomic>
#include <cstdio>
#include <cmath>

namespace {
void format_distance(int mm, char* text) {
    if (mm == PROS_ERR || mm < 0) std::snprintf(text, 6, "  ERR");
    else if (mm == 9999) std::snprintf(text, 6, " NONE");
    else if (mm < 20) std::snprintf(text, 6, "  LOW");
    else std::snprintf(text, 6, "%5.2f", mm / 25.4);
}

struct Auton {
    const char* name;
    void (*run)();
};

const Auton autons[] = {
    {"Red", auton_steps},
    {"Blue", auton_steps}
};
constexpr int count = sizeof(autons) / sizeof(autons[0]);
std::atomic<int> selected{1};
std::atomic<int> active_selection{1};
std::atomic<unsigned int> selection_shown_at{0};
std::atomic<bool> run_requested{false};
std::atomic<bool> running{false};

void show_selection() {
    selection_shown_at = pros::millis();
    pros::lcd::set_text(1, "Auton");
    pros::lcd::set_text(3, std::string("> ") + autons[selected.load()].name + " <");
    pros::lcd::set_text(7, "Previous       Run       Next");
}

void previous() {
    if (running) return;
    selected = (selected.load() + count - 1) % count;
    show_selection();
}

void next() {
    if (running) return;
    selected = (selected.load() + 1) % count;
    show_selection();
}
void request_run() {
    if (!running && !pros::competition::is_connected() &&
        !pros::competition::is_disabled()) run_requested = true;
}
}

void initialize_selector() {
    pros::lcd::initialize();
    pros::lcd::register_btn0_cb(previous);
    pros::lcd::register_btn1_cb(request_run);
    pros::lcd::register_btn2_cb(next);
    show_selection();
    static pros::Task coordinates([] {
        pros::Controller controller(pros::E_CONTROLLER_MASTER);
        int controller_line = 0;
        int display_tick = 0;
        while (true) {
            if (controller.is_connected()) {
                const bool change_color = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
                if (change_color && !running) next();
            }
            if (++display_tick < 5) {
                pros::delay(20);
                continue;
            }
            display_tick = 0;
            const auto pose = robot::chassis().getPose();
            pros::lcd::print(0, "X (right+): %.2f in", pose.x);
            pros::lcd::print(2, "Y (forward+): %.2f in", pose.y);
            pros::lcd::print(5, "Heading: %.1f deg", pose.theta);
            if (controller.is_connected()) {
                if (controller_line == 0) {
                    if (((pros::millis() - selection_shown_at.load()) / 2000) % 2 == 0)
                        controller.print(0, 0, "Auton: %-8s", autons[selected.load()].name);
                    else
                        controller.print(0, 0, "X%6.1f Y%6.1f", pose.x, pose.y);
                }
                if (controller_line == 1) {
                    char cup[6];
                    format_distance(robot::cup_sensor.get(), cup);
                    if (!robot::imu.is_installed()) controller.print(1, 0, "IMU8 missing   ");
                    else if (robot::imu.is_calibrating()) controller.print(1, 0, "IMU8 calibrate ");
                    else if (!std::isfinite(robot::imu.get_rotation()))
                        controller.print(1, 0, "IMU8 read error");
                    else if (!std::isfinite(pose.theta))
                        controller.print(1, 0, "IMU OK HDG ERR ");
                    else {
                        const float heading = std::fmod(std::fmod(pose.theta, 360.0f) + 360.0f, 360.0f);
                        controller.print(1, 0, "HDG%5.1f C%5s", heading, cup);
                    }
                }
                if (controller_line == 2) {
                    char front[6], right[6];
                    format_distance(robot::wall_sensor.get(), front);
                    format_distance(robot::right_wall_sensor.get(), right);
                    controller.print(2, 0, "F:%s R:%s", front, right);
                }
                controller_line = (controller_line + 1) % 3;
            }
            pros::delay(20);
        }
    });
}

void run_selected_auton() {
    active_selection = selected.load();
    running = true;
    run_requested = false;
    const auto run = autons[active_selection.load()].run;
    if (run) run();
    running = false;
}

bool take_run_request() { return run_requested.exchange(false); }

int auton_toggle_degrees() {
    const int color = running ? active_selection.load() : selected.load();
    return color == 0 ? 540 : 1080;
}
