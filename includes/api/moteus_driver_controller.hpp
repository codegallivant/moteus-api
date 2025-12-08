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

    MoteusDriverController(int moteus_id = 1, std::string socketcan_iface = "can0", int socketcan_ignore_errors = 0, int socketcan_disable_brs = 0) {

        mjbots::moteus::Controller::Options options;
        options.id = moteus_id;
        options.default_query = true; // Whether to get read response after writing for every command

        std::vector<std::string> transport_args = {
            "--socketcan-iface", socketcan_iface,
            "--socketcan-ignore-errors", std::to_string(socketcan_ignore_errors),
            "--can-disable-brs", std::to_string(socketcan_disable_brs)
        };
        auto transport = mjbots::moteus::Controller::MakeSingletonTransport(transport_args);
        options.transport = transport;

        // options.position_format.kp_scale = mjbots::moteus::kFloat;
        // options.position_format.kd_scale = mjbots::moteus::kFloat;
        // options.position_format.ilimit_scale = mjbots::moteus::kFloat;
        // options.position_format.feedforward_torque = mjbots::moteus::kFloat;

        this->controller = std::make_unique<mjbots::moteus::Controller>(options);
        this->controller->SetStop();

    }

    mjbots::moteus::Query::Format getReadFormat(motor_state readState) {
        mjbots::moteus::Query::Format q_com;
        for (auto it = readState.begin(); it != readState.end(); it++) {
            const std::string& key = it->first;
            if (key == "position") {
                q_com.position = mjbots::moteus::Resolution::kFloat;
            } else if (key == "abs_position") {
                q_com.abs_position = mjbots::moteus::Resolution::kFloat;
            } else if (key == "aux1_gpio") {
                q_com.aux1_gpio = mjbots::moteus::Resolution::kFloat;
            } else if (key == "aux2_gpio") {
                q_com.aux2_gpio = mjbots::moteus::Resolution::kFloat;
            } else if (key == "d_current") {
                q_com.d_current = mjbots::moteus::Resolution::kFloat;
            } else if (key == "extra.register_number") {
                q_com.extra[0].register_number = mjbots::moteus::Resolution::kInt16;
            } else if (key == "extra.value") {
                q_com.extra[0].resolution = mjbots::moteus::Resolution::kInt16;
            } else if (key == "fault") {
                q_com.fault = mjbots::moteus::Resolution::kInt8;
            } else if (key == "home_state") {
                q_com.home_state = mjbots::moteus::Resolution::kInt16;
            } else if (key == "mode") {
                q_com.mode = mjbots::moteus::Resolution::kInt8;
            } else if (key == "motor_temperature") {
                q_com.motor_temperature = mjbots::moteus::Resolution::kFloat;
            } else if (key == "power") {
                q_com.power = mjbots::moteus::Resolution::kFloat;
            } else if (key == "q_current") {
                q_com.q_current = mjbots::moteus::Resolution::kFloat;
            } else if (key == "temperature") {
                q_com.temperature = mjbots::moteus::Resolution::kFloat;
            } else if (key == "torque") {
                q_com.torque = mjbots::moteus::Resolution::kFloat;
            } else if (key == "trajectory_complete") {
                q_com.trajectory_complete = mjbots::moteus::Resolution::kInt16;
            } else if (key == "velocity") {
                q_com.velocity = mjbots::moteus::Resolution::kFloat;
            } else if (key == "voltage") {
                q_com.voltage = mjbots::moteus::Resolution::kFloat;
            } else {
                std::cout << "WARNING: Unrecognized keyword " << key << std::endl;
            }
        }
        return q_com;
    }

    void populateReadState(motor_state& readState, mjbots::moteus::Query::Result values) {
        for (auto it = readState.begin(); it != readState.end(); it++) {
            const std::string& key = it->first;
            if (key == "position") {
                it->second = values.position;
            } else if (key == "abs_position") {
                it->second = values.abs_position;
            } else if (key == "aux1_gpio") {
                it->second = values.aux1_gpio;
            } else if (key == "aux2_gpio") {
                it->second = values.aux2_gpio;
            } else if (key == "d_current") {
                it->second = values.d_current;
            } else if (key == "fault") {
                it->second = values.fault;
            } else if (key == "home_state") {
                it->second = static_cast<float>(values.home_state);
            } else if (key == "mode") {
                it->second = static_cast<float>(values.mode);
            } else if (key == "motor_temperature") {
                it->second = values.motor_temperature;
            } else if (key == "power") {
                it->second = values.power;
            } else if (key == "q_current") {
                it->second = values.q_current;
            } else if (key == "temperature") {
                it->second = values.temperature;
            } else if (key == "torque") {
                it->second = values.torque;
            } else if (key == "trajectory_complete") {
                it->second = values.trajectory_complete;
            } else if (key == "velocity") {
                it->second = values.velocity;
            } else if (key == "voltage") {
                it->second = values.voltage;
            } else {
                std::cout << "WARNING: Unrecognized keyword " << key << std::endl;
            }
        }
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
        auto qcom = getReadFormat(readState);
        const auto result = this->controller->SetQuery(&qcom);
        if(!result) {
            return false;
        }
        populateReadState(readState, result->values);
        return true;
    }

    bool write(motor_state state, motor_state& readState) {
        mjbots::moteus::PositionMode::Format res; // Format in which command is given (i.e. to write in)
        res.position = mjbots::moteus::Resolution::kIgnore;
        res.velocity = mjbots::moteus::Resolution::kIgnore;
        for (auto it = state.begin(); it != state.end(); ++it) {
            if (it->first == "position") {
                res.position = mjbots::moteus::Resolution::kFloat;
            } else if (it->first == "velocity") {
                res.velocity = mjbots::moteus::Resolution::kFloat;
                res.position = mjbots::moteus::Resolution::kFloat;
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
        auto qcom = getReadFormat(readState);
        auto result = this->controller->SetPosition(createCommand(state), &res, &qcom);  
        if(!result) {
            return false;
        } 
        populateReadState(readState, result->values);
        return true;
    }

    bool writeDuration(motor_state state, motor_state& responseState, int duration_ms) {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool status;
        while (true) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            if (elapsed_time.count() >= duration_ms) {
                break;
            }
            status = this->write(state, responseState);
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
        this->controller->SetStop();
    }
};