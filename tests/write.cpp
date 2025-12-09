#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <limits>
#include <vector>
#include <optional>
#include <csignal>
#include <sstream>

#include "api/moteus_driver_controller.hpp"
#include "lib/cxxopts.hpp" 


void signalHandler(int signum) {
    MoteusDriverController::destroyAll();
}

int main(int argc, char** argv) {

    std::signal(SIGINT, signalHandler);

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
        ("moteus-id", "Moteus ID(s)", cxxopts::value<std::string>()->default_value("1"))
        ("socketcan-iface", "SocketCAN Interface", cxxopts::value<std::string>()->default_value("can0"))
        ("socketcan-ignore-errors", "Ignore SocketCAN errors", cxxopts::value<int>()->default_value("0"))
        ("can-disable-brs", "Disable CAN BRS (bit-rate switching)", cxxopts::value<int>()->default_value("0"))

        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    // Parse moteus-id as space separated list of ints
    std::string moteus_id_str = result["moteus-id"].as<std::string>();
    std::istringstream id_stream(moteus_id_str);
    std::vector<int> moteus_ids;
    int id_val;
    while (id_stream >> id_val) {
        moteus_ids.push_back(id_val);
    }

    if (moteus_ids.empty()) {
        std::cerr << "No valid moteus IDs provided." << std::endl;
        return 1;
    }

    std::string socketcan_iface = result["socketcan-iface"].as<std::string>();
    int socketcan_ignore_errors = result["socketcan-ignore-errors"].as<int>();
    int socketcan_disable_brs = result["can-disable-brs"].as<int>();

    struct ControllerEntry {
        int id;
        MoteusDriverController* mc;
    };

    std::vector<ControllerEntry> controllers;
    controllers.reserve(moteus_ids.size());

    // Instantiate K controllers
    for (int mid : moteus_ids) {
        MoteusDriverController* mc = MoteusDriverController::create(
            mid,
            socketcan_iface,
            socketcan_ignore_errors,
            socketcan_disable_brs
        );
        if (!mc) {
            std::cerr << "Failed to create controller for moteus ID " << mid << std::endl;
            return 1;
        }
        controllers.push_back({mid, mc});
    }

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
        std::cout << "Sending command..." << std::endl;

        if (result.count("duration-ms")) {
            int duration = result["duration-ms"].as<int>();            
            auto start_time = std::chrono::high_resolution_clock::now();
            bool status;
            int count = 0;
            int elapsed_time_count = 0;
            while (true) {
                if (elapsed_time_count >= duration) {
                    break;
                }
                auto start_time = std::chrono::high_resolution_clock::now();
                for (auto& entry : controllers) {
                    // std::cout << "  Moteus ID " << entry.id
                    //         << " for " << duration << " ms" << std::endl;
                    // entry.mc->writeDuration(writeState, duration);
                    entry.mc->write(writeState);
                }
                auto end_time = std::chrono::high_resolution_clock::now();
                count += controllers.size();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                elapsed_time_count += elapsed_time.count();
            }
            std::cout <<  "Control Frequency (successful writes per second across all controllers): " << count/(static_cast<double>(elapsed_time_count)/1000) << std::endl;
        } else {
            bool overall_status = true;

            // Apply write to all controllers one after the other
            for (auto& entry : controllers) {
                // Per-controller readback
                MoteusDriverController::motor_state readState = {
                    {"position", std::nullopt},
                    // add more if you later want them
                };

                bool status = entry.mc->write(writeState, readState);
                if (status) {
                    std::cout << "\n=== Moteus ID " << entry.id << " ===" << std::endl;
                    MoteusDriverController::displayState(readState);
                } else {
                    std::cerr << "Write failed for moteus ID " << entry.id << std::endl;
                    overall_status = false;
                }
            }

            if (!overall_status) {
                return 1;
            }
        }
    }
    return 0;
}
