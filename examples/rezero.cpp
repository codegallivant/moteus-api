// Tests setting zero offset
#include "moteus_api.hpp" // Make sure to have the relevant moteus_api.hpp, moteus_api.cpp and moteus drivers present at appropriate paths
#include <csignal>
#include <chrono>


void signal_handler(int signal) {
    // Cleanly stops controllers and releases memory
    MoteusAPI::Controller::destroyAll();
}

int main() {
    int moteus_id = 11;
    std::string can_interface = "can0";

    // Important to ensure all servos are stopped upon any termination signal 
    std::signal(SIGINT, signal_handler);

    MoteusAPI::Controller* controller = MoteusAPI::Controller::create(moteus_id, can_interface);

    // Specify which parameters should be read
    MoteusAPI::ReadState rs({
        .position = true,
    });

    // Velocity control
    MoteusAPI::CommandState cs({
        .kp_scale = 0.1,
        .kd_scale = 0.1,
        .position = std::numeric_limits<double>::quiet_NaN(),
        .velocity = 2.0,
    });

    double max_motor_current = 40; // 40A max continuous phase current
    double max_moteus_current = 12; // 12A max continuous phase current
    double max_d_current = 5; // d_current should be near 0
    double violation_limit_ms = 100; // max ms of violation allowed

    controller->configureSafety(max_motor_current, max_moteus_current, max_d_current, violation_limit_ms);
    controller->writeDuration(cs, 2000);

    // Read operation on controller
    controller->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();

    std::cout << std::endl;

    // Rezero
    std::cout << "Setting zero offset.." << std::endl;
    bool status = controller->setZeroOffset();
    if(status) {
        std::cout << "Success" << std::endl;
    } else {
        std::cout << "Failed" << std::endl;
    }
    std::cout << std::endl;

    // Read operation on controller
    controller->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();
}