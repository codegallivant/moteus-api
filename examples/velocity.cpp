// Tests velocity control
#include "moteus_api.hpp" // Make sure to have the relevant moteus_api.hpp, moteus_api.cpp and moteus drivers present at appropriate paths
#include <csignal>
#include <chrono>


void signal_handler(int signal) {
    // Cleanly stops controllers and releases memory
    MoteusAPI::Controller::destroyAll();
}

int main() {
    // Important to ensure all servos are stopped upon any termination signal 
    std::signal(SIGINT, signal_handler);

    MoteusAPI::Controller* controller = MoteusAPI::Controller::create(11, "can0");

    // Specify which parameters should be read
    MoteusAPI::ReadState rs({
        .velocity = true,
    });

    // Pure velocity control
    MoteusAPI::CommandState cs({
        // .feedforward_torque = 0.1,
        .kp_scale = 0.0,
        .kd_scale = 0.1,
        .maximum_torque = 1.67,
        // .watchdog_timeout = 200
        .velocity = 2.0
    });

    double max_motor_current = 40; // 40A max continuous phase current
    double max_moteus_current = 12; // 12A max continuous phase current
    double max_d_current = 5; // d_current should be near 0
    double violation_limit_ms = 100; // max ms of violation allowed

    controller->setZeroOffset();
    controller->configureSafety(max_motor_current, max_moteus_current, max_d_current, violation_limit_ms);
    controller->writeDuration(cs, 5000, true, true);

    std::cout << std::endl;
}