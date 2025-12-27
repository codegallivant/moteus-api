// Tests reading data
#include "moteus_api.hpp" // Make sure to have the relevant moteus_api.hpp, moteus_api.cpp and moteus drivers present at appropriate paths
#include <csignal>
#include <chrono>


void signal_handler(int signal) {
    // Cleanly stops controllers and releases memory
    MoteusAPI::Controller::destroyAll();
}

int main() {
    int moteus_id = 11;
    std::string socketcan_iface = "can0";
    // Important to ensure all servos are stopped upon any termination signal 
    std::signal(SIGINT, signal_handler);

    MoteusAPI::Controller* controller = MoteusAPI::Controller::create(moteus_id, socketcan_iface);

    // Specify which parameters should be read
    // Ensure resultant size is under CAN FD frame limit (otherwise nan values will be returned)
    MoteusAPI::ReadState rs({
        .abs_position        = true,
        .aux1_gpio           = true,
        .aux2_gpio           = true,
        .d_current           = true,
        .fault               = true,
        .home_state          = true,
        .mode                = true,
        .motor_temperature   = true,
        .position            = true, // Enabling more params causes nan return
        // .power               = true,
        // .q_current           = true,
        // .temperature         = true,
        // .torque              = true,
        // .trajectory_complete = true,
        // .velocity            = true,
        // .voltage             = true,
    });


    controller->read(rs);
    rs.display();

    std::cout << std::endl;
}