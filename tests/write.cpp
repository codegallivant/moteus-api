#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <limits>
#include <vector>
#include <optional>
#include <csignal>
#include <sstream>
#include <unordered_map>
#include <chrono>

#include "moteus/api/moteus_api.hpp"
#include "lib/cxxopts.hpp"

void signalHandler(int) {
    MoteusAPI::Controller::destroyAll();
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    cxxopts::Options options("MoteusWrite", "Write commands Moteus via CLI");
    options.allow_unrecognised_options();

    options.add_options()
        ("p,position", "Position (revolutions)", cxxopts::value<std::string>())
        ("v,velocity", "Velocity (rev/s)", cxxopts::value<std::string>())
        ("stop-position", "Stop Position", cxxopts::value<std::string>())

        ("accel-limit", "Acceleration Limit", cxxopts::value<std::string>())
        ("velocity-limit", "Velocity Limit", cxxopts::value<std::string>())
        ("max-torque", "Maximum Torque", cxxopts::value<std::string>())

        ("feedforward-torque", "Feedforward Torque", cxxopts::value<std::string>())
        ("kp-scale", "Kp Scale", cxxopts::value<std::string>())
        ("kd-scale", "Kd Scale", cxxopts::value<std::string>())
        ("ilimit-scale", "I-Limit Scale", cxxopts::value<std::string>())

        ("watchdog", "Watchdog Timeout", cxxopts::value<std::string>())

        ("duration-ms", "Duration in milliseconds", cxxopts::value<int>())

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
        MoteusAPI::Controller* mc;
    };

    std::vector<ControllerEntry> controllers;
    controllers.reserve(moteus_ids.size());

    for (int mid : moteus_ids) {
        MoteusAPI::Controller* mc = MoteusAPI::Controller::create(
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

    std::unordered_map<std::string, double> writeValues;

    auto add = [&](const std::string& cli_flag, const std::string& map_key) {
        if (result.count(cli_flag)) {
            std::string val = result[cli_flag].as<std::string>();
            std::string val_lower = val;
            std::transform(
                val_lower.begin(),
                val_lower.end(),
                val_lower.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); }
            );

            if (val_lower == "nan") {
                writeValues[map_key] = std::numeric_limits<double>::quiet_NaN();
            } else {
                try {
                    writeValues[map_key] = std::stod(val);
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing argument '" << cli_flag
                              << "': " << e.what() << std::endl;
                }
            }
        }
    };

    add("position",          "position");
    add("velocity",          "velocity");
    add("stop-position",     "stop_position");

    add("accel-limit",       "accel_limit");
    add("velocity-limit",    "velocity_limit");
    add("max-torque",        "maximum_torque");

    add("feedforward-torque","feedforward_torque");
    add("kp-scale",          "kp_scale");
    add("kd-scale",          "kd_scale");
    add("ilimit-scale",      "ilimit_scale");

    add("watchdog",          "watchdog_timeout");

    if (writeValues.empty()) {
        std::cout << "No motor commands provided. (Use --help to see options)" << std::endl;
        return 0;
    }

    std::cout << "Sending command..." << std::endl;

    auto getOpt = [&](const char* key) -> std::optional<double> {
        auto it = writeValues.find(key);
        if (it == writeValues.end()) {
            return std::nullopt;
        }
        return it->second;
    };

    MoteusAPI::CommandState cs({
        .accel_limit        = getOpt("accel_limit"),
        .feedforward_torque = getOpt("feedforward_torque"),
        .ilimit_scale       = getOpt("ilimit_scale"),
        .kp_scale           = getOpt("kp_scale"),
        .kd_scale           = getOpt("kd_scale"),
        .maximum_torque     = getOpt("maximum_torque"),
        .position           = getOpt("position"),
        .stop_position      = getOpt("stop_position"),
        .watchdog_timeout   = getOpt("watchdog_timeout"),
        .velocity           = getOpt("velocity"),
        .velocity_limit     = getOpt("velocity_limit"),
    });

    if (result.count("duration-ms")) {
        int duration = result["duration-ms"].as<int>();
        using clock = std::chrono::high_resolution_clock;
        using ms = std::chrono::milliseconds;

        ms elapsed_time(0);
        std::size_t count = 0;

        while (elapsed_time.count() < duration) {
            auto loop_start = clock::now();
            for (auto& entry : controllers) {
                entry.mc->write(cs);
                ++count;
            }
            auto loop_end = clock::now();
            elapsed_time += std::chrono::duration_cast<ms>(loop_end - loop_start);
        }

        double seconds = static_cast<double>(elapsed_time.count()) / 1000.0;
        if (seconds > 0.0) {
            std::cout << "Control Frequency (successful writes per second across all controllers): "
                      << static_cast<double>(count) / seconds << std::endl;
        } else {
            std::cout << "Duration too short to measure control frequency." << std::endl;
        }
    } else {
        bool overall_status = true;

        for (auto& entry : controllers) {
            MoteusAPI::ReadState rs({
                .position = true
            });

            bool status = entry.mc->write(cs, rs);
            if (status) {
                std::cout << "\n=== Moteus ID " << entry.id << " ===" << std::endl;
                rs.display();
            } else {
                std::cerr << "Write failed for moteus ID " << entry.id << std::endl;
                overall_status = false;
            }
        }

        if (!overall_status) {
            return 1;
        }
    }

    return 0;
}
