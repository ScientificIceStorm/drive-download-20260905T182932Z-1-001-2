#include "main.h"
#include "auton.hpp"
#include "auton_selector.hpp"
#include <atomic>

namespace {
struct Auton {
    const char* name;
    void (*run)();
};

const Auton autons[] = {
    {"New bot", auton_steps},
    {"None", nullptr}
};
constexpr int count = sizeof(autons) / sizeof(autons[0]);
std::atomic<int> selected{0};
std::atomic<bool> run_requested{false};
std::atomic<bool> running{false};

void show_selection() {
    pros::lcd::set_text(1, "Auton");
    pros::lcd::set_text(3, std::string("> ") + autons[selected.load()].name + " <");
    pros::lcd::set_text(7, "Previous       Run       Next");
}

void previous() {
    selected = (selected.load() + count - 1) % count;
    show_selection();
}

void next() {
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
        while (true) {
            const auto pose = robot::chassis().getPose();
            pros::lcd::print(0, "X (right+): %.2f in", pose.x);
            pros::lcd::print(2, "Y (forward+): %.2f in", pose.y);
            pros::lcd::print(5, "Heading: %.1f deg", pose.theta);
            pros::delay(100);
        }
    });
}

void run_selected_auton() {
    running = true;
    run_requested = false;
    const auto run = autons[selected.load()].run;
    if (run) run();
    running = false;
}

bool take_run_request() { return run_requested.exchange(false); }
