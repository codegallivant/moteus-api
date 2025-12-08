#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <limits>
#include <vector>
#include <optional>

#include "api/moteus_driver_controller.hpp"
#include "lib/cxxopts.hpp" 

int main(int argc, char** argv) {
    cxxopts::Options options("MoteusWrite", "Write commands Moteus via CLI");

    options.allow_unrecognised_options();

    options.add_options()
        // Motion Commands
        ("p,position", "Position (revolutions)", cxxopts::value<std::string>())
        ("v,velocity", "Velocity (rev/s)", cxxopts::value<std::string>())
        ("stop-position", "Stop Position", cxxopts::value<std::string>())

        // Limits & Dynamics
        ("accel-limit", "Acceleration Limit", cxxopts::value<std::string>())
        ("velocity-limit", "Velocity Limit", cxxopts::value<std::string>())
        ("max-torque", "Maximum Torque", cxxopts::value<std::string>())
        
        // Tuning / Gains
        ("feedforward-torque", "Feedforward Torque", cxxopts::value<std::string>())
        ("kp-scale", "Kp Scale", cxxopts::value<std::string>())
        ("kd-scale", "Kd Scale", cxxopts::value<std::string>())
        ("ilimit-scale", "I-Limit Scale", cxxopts::value<std::string>())
        
        // Safety
        ("watchdog", "Watchdog Timeout", cxxopts::value<std::string>())
        
        // Timing (Keep as int)
        ("duration-ms", "Duration in milliseconds", cxxopts::value<int>())

        // Transport args
        ("moteus-id", "Moteus ID", cxxopts::value<int>()->default_value("1"))
        ("socketcan-iface", "SocketCAN Interface", cxxopts::value<std::string>()->default_value("can0"))
        ("socketcan-ignore-errors", "Ignore SocketCAN errors", cxxopts::value<int>()->default_value("0"))
        ("can-disable-brs", "Disable CAN BRS (bit-rate switching)", cxxopts::value<int>()->default_value("0"))

        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    int moteus_id = result["moteus-id"].as<int>();
    std::string socketcan_iface = result["socketcan-iface"].as<std::string>();
    int socketcan_ignore_errors = result["socketcan-ignore-errors"].as<int>();
    int socketcan_disable_brs = result["can-disable-brs"].as<int>();

    auto mc = std::make_unique<MoteusDriverController>(
        moteus_id,
        socketcan_iface,
        socketcan_ignore_errors,
        socketcan_disable_brs
    );

    MoteusDriverController::motor_state writeState;

    auto add = [&](std::string cli_flag, std::string map_key) {
        if (result.count(cli_flag)) {
            std::string val = result[cli_flag].as<std::string>();            
            std::string val_lower = val;
            std::transform(val_lower.begin(), val_lower.end(), val_lower.begin(),
                [](unsigned char c){ return std::tolower(c); });

            if (val_lower == "nan") {
                writeState[map_key] = std::numeric_limits<double>::quiet_NaN();
            } else {
                try {
                    writeState[map_key] = std::stod(val);
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing argument '" << cli_flag
                              << "': " << e.what() << std::endl;
                }
            }
        }
    };

    // args mapping
    add("position",         "position");
    add("velocity",         "velocity");
    add("stop-position",    "stop_position");
    
    add("accel-limit",      "accel_limit");
    add("velocity-limit",   "velocity_limit");
    add("max-torque",       "maximum_torque");

    add("feedforward-torque","feedforward_torque");
    add("kp-scale",         "kp_scale");
    add("kd-scale",         "kd_scale");
    add("ilimit-scale",     "ilimit_scale");

    add("watchdog",         "watchdog_timeout");

    if (writeState.empty()) {
        std::cout << "No motor commands provided. (Use --help to see options)" << std::endl;
    } else {
        // Sample read params
        MoteusDriverController::motor_state readState = {
            {"position", std::nullopt},
            {"velocity", std::nullopt},
            {"torque", std::nullopt},
            {"temperature", std::nullopt},
            {"voltage", std::nullopt},
        };

        std::cout << "Sending command..." << std::endl;

        if (result.count("duration-ms")) {
            int duration = result["duration-ms"].as<int>();
            
            if (mc->writeDuration(writeState, readState, duration)) {
                std::cout << "Duration command completed." << std::endl;
                MoteusDriverController::displayState(readState);
            } else {
                std::cerr << "Write failed" << std::endl;
            }
        } else {
            if (mc->write(writeState, readState)) {
                MoteusDriverController::displayState(readState);
            } else {
                std::cerr << "Write failed" << std::endl;
            }
        }
    }

    return 0;
}
