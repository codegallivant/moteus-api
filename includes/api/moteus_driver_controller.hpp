#pragma once
#include "moteus_drivers/moteus.h"
#include <optional>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <string>


class MoteusDriverController {
public:
    typedef std::unordered_map<std::string, std::optional<double>> motor_state; 
    std::unique_ptr<mjbots::moteus::Controller> controller;
    mjbots::moteus::Query::Format q_com; // Resolution to receive reply

    MoteusDriverController(int motor_id = 1) {

        mjbots::moteus::Controller::Options options;
        options.id = motor_id;
        options.default_query = true; // Whether to get read response after writing for every command
        // options.position_format.kp_scale = mjbots::moteus::kFloat;
        // options.position_format.kd_scale = mjbots::moteus::kFloat;
        // options.position_format.ilimit_scale = mjbots::moteus::kFloat;
        // options.position_format.feedforward_torque = mjbots::moteus::kFloat;

        q_com.position = mjbots::moteus::Resolution::kFloat;
        q_com.abs_position = mjbots::moteus::Resolution::kFloat;
        q_com.aux1_gpio = mjbots::moteus::Resolution::kFloat;
        q_com.aux2_gpio = mjbots::moteus::Resolution::kFloat;
        q_com.d_current = mjbots::moteus::Resolution::kFloat;
        // q_com.extra->register_number = mjbots::moteus::Resolution::kInt16;
        // q_com.extra->resolution = mjbots::moteus::Resolution::kInt16;
        q_com.fault = mjbots::moteus::Resolution::kInt8;
        q_com.home_state = mjbots::moteus::Resolution::kInt16;
        q_com.mode = mjbots::moteus::Resolution::kInt8;
        q_com.motor_temperature = mjbots::moteus::Resolution::kFloat;
        q_com.power = mjbots::moteus::Resolution::kFloat;
        q_com.q_current = mjbots::moteus::Resolution::kFloat;
        q_com.temperature = mjbots::moteus::Resolution::kFloat;
        q_com.torque = mjbots::moteus::Resolution::kFloat;
        q_com.trajectory_complete = mjbots::moteus::Resolution::kInt16;
        q_com.velocity = mjbots::moteus::Resolution::kFloat;
        q_com.voltage = mjbots::moteus::Resolution::kFloat;

        this->controller = std::make_unique<mjbots::moteus::Controller>(options);
        this->controller->SetStop();
    }

    void modifyCommand(mjbots::moteus::PositionMode::Command& cmd, motor_state state) {
        for (auto it = state.begin(); it != state.end(); ++it) {
            // std::cout << "Key: " << it->first << std::endl;
            double value;
            if(it->second.has_value()) {
                value = it->second.value();
            } else {
                continue;
            }
            if(it->first == "velocity") {
                cmd.velocity = value;
            } else if(it->first == "position") {
                cmd.position = value;
            } else if(it->first == "feedforward_torque") {
                cmd.feedforward_torque = value;
            } else if(it->first == "kp_scale") {
                cmd.kp_scale = value;
            } else if(it->first == "kd_scale") {
                cmd.kd_scale = value;
            } else if(it->first == "ilimit_scale") {
                cmd.ilimit_scale = value;
            } else if(it->first == "maximum_torque") {
                cmd.maximum_torque = value;
            } else if(it->first == "stop_position") {
                cmd.stop_position = value;
            } else if(it->first == "velocity_limit") {
                cmd.velocity_limit = value;
            } else if(it->first == "accel_limit") {
                cmd.accel_limit = value;
            } else if(it->first == "watchdog_timeout") {
                cmd.watchdog_timeout = value;
            } else {
                std::cout << "WARNING: Unrecognized keyword " << it->first << std::endl;
            }
        }
    }

    mjbots::moteus::PositionMode::Command createCommand(motor_state state) {
        mjbots::moteus::PositionMode::Command cmd;
        modifyCommand(cmd, state);
        return cmd;
    }

    bool read(motor_state& readState) {
        const auto result = this->controller->SetQuery(&q_com);
        if(!result) {
            return false;
        }
        readState = {
            {"position", result->values.position},
            {"abs_position", result->values.abs_position},
            {"aux1_gpio", result->values.aux1_gpio},
            {"aux2_gpio", result->values.aux2_gpio},
            {"d_current", result->values.d_current},
            // {"extra.register_number", result->values.extra[0].register_number},
            // {"extra.value", result->values.extra[0].value},
            {"fault", result->values.fault},
            {"home_state", static_cast<float>(result->values.home_state)},
            {"mode", static_cast<float>(result->values.mode)},
            {"motor_temperature", result->values.motor_temperature},
            {"power", result->values.power},
            {"q_current", result->values.q_current},
            {"temperature", result->values.temperature},
            {"torque", result->values.torque},
            {"trajectory_complete", result->values.trajectory_complete},
            {"velocity", result->values.velocity},
            {"voltage", result->values.voltage}
        };
        // readState = {
        //     {"velocity", result->values.velocity},
        //     {"position", result->values.position},
        //     {"torque", result->values.torque},
        //     {"q_current", result->values.q_current},
        //     {"d_current", result->values.d_current},
        //     {"voltage", result->values.voltage},
        //     {"temperature", result->values.temperature},
        //     {"fault", result->values.fault},
        //     {"mode",  static_cast<float>(result->values.mode)},
        //     {"trajectory_complete", static_cast<int>(result->values.trajectory_complete)},
        //     {"home_state", static_cast<float>(result->values.)}
        // };
        return true;
    }

    bool write(motor_state state, motor_state& responseState) {
        mjbots::moteus::PositionMode::Format res; // Format in which command is given (i.e. to write in)
        res.position = mjbots::moteus::Resolution::kIgnore;
        res.velocity = mjbots::moteus::Resolution::kIgnore;
        for (auto it = state.begin(); it != state.end(); ++it) {
            if (it->first == "position") {
                res.position = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "velocity") {
                res.velocity = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "feedforward_torque") {
                res.feedforward_torque = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "kp_scale") {
                res.kp_scale = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "kd_scale") {
                res.kd_scale = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "maximum_torque") {
                res.maximum_torque = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "stop_position") {
                res.stop_position = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "watchdog_timeout") {
                res.watchdog_timeout = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "velocity_limit") {
                res.velocity_limit = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "accel_limit") {
                res.accel_limit = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "ilimit_scale") {
                res.ilimit_scale = mjbots::moteus::Resolution::kFloat;
            } else {
                std::cout << "WARNING: Unrecognized keyword " << it->first << std::endl;
            }
        }
        auto result = this->controller->SetPosition(createCommand(state), &res, &q_com);  
        if(!result) {
            return false;
        } 
        responseState = {
            {"position", result->values.position},
            {"abs_position", result->values.abs_position},
            {"aux1_gpio", result->values.aux1_gpio},
            {"aux2_gpio", result->values.aux2_gpio},
            {"d_current", result->values.d_current},
            // {"extra.register_number", result->values.extra->register_number},
            // {"extra.value", result->values.extra->value},
            {"fault", result->values.fault},
            {"home_state", static_cast<float>(result->values.home_state)},
            {"mode", static_cast<float>(result->values.mode)},
            {"motor_temperature", result->values.motor_temperature},
            {"power", result->values.power},
            {"q_current", result->values.q_current},
            {"temperature", result->values.temperature},
            {"torque", result->values.torque},
            {"trajectory_complete", result->values.trajectory_complete},
            {"velocity", result->values.velocity},
            {"voltage", result->values.voltage}
        };
        return true;
    }

    bool writeDuration(motor_state state, motor_state& responseState, int duration_ms, int interval_ms) {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool status;
        while (true) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            if (elapsed_time.count() >= duration_ms) {
                // std::cout << "Finished after " << elapsed_time.count() << " milliseconds." << std::endl;
                break;
            }
            status = this->write(state, responseState);
            // std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
        return status;
    }

    static void displayState(const std::unordered_map<std::string, std::optional<double>>& m) {
        std::cout << "\nmotor_state" << std::endl;
        for (const auto& [key, opt_val] : m) {
            std::cout << key << ": ";
            if (opt_val.has_value()) {
                std::cout << *opt_val;
            } else {
                std::cout << "null";
            }
            std::cout << "\n";
        }
    }

    ~MoteusDriverController() {
        // std::cout << "Destructor Called" << std::endl;
        this->controller->SetStop();
    }
};