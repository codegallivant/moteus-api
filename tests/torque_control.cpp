#include "api/moteus_driver_controller.hpp"


int main(int argc, char** argv) {
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);
    auto mc = std::make_unique<MoteusDriverController>();
    MoteusDriverController::motor_state responseState;
    bool status;
    MoteusDriverController::motor_state writeState;
    writeState = {
        {"position", std::numeric_limits<double>::quiet_NaN()},
        {"velocity" , 0.0},
        {"kp_scale", 0.0},
        {"kd_scale", 0.0},
        {"ilimit_scale", 0.0},
        {"feedforward_torque", 0.1},
    };
    status = mc->writeDuration(writeState, responseState, 5000, 100);
    if(status) {
        MoteusDriverController::displayState(responseState);
    } else {
        std::cout << "Failed to write" << std::endl;
    }
/*
Examples

Velocity Control
cmd.position = std::numeric_limits<double>::quiet_NaN();
cmd.velocity = 2.0;  // 2 revolutions/second

Torque Control
cmd.position = std::numeric_limits<double>::quiet_NaN();
cmd.velocity = 0.0;
cmd.kp_scale = 0.0;
cmd.kd_scale = 0.0;
cmd.ilimit_scale = 0.0;
cmd.feedforward_torque = 0.1;

Constant Acceleration Trajectories¶
cmd.position = 1.0;
cmd.velocity = 0.0;
cmd.accel_limit = 2.0;
cmd_velocity_limit = 0.5;
*/
}