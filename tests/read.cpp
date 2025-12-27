#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <sstream>
#include <chrono>
#include <unordered_map>

#include "moteus_api.hpp"
#include "cxxopts.hpp"

void signalHandler(int) {
    MoteusAPI::Controller::destroyAll();
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    cxxopts::Options options("MoteusRead", "Read Moteus Controller State");
    options.allow_unrecognised_options();

    options.add_options()
        ("p,position", "Read Position")
        ("v,velocity", "Read Velocity")
        ("t,torque", "Read Torque")
        ("temperature", "Read Board Temperature")
        ("abs-position", "Read Absolute Position")
        ("aux1-gpio", "Read Aux1 GPIO")
        ("aux2-gpio", "Read Aux2 GPIO")
        ("d-current", "Read D Current")
        ("q-current", "Read Q Current")
        ("fault", "Read Fault Status")
        ("home-state", "Read Home State")
        ("mode", "Read Mode")
        ("motor-temperature", "Read Motor Temperature")
        ("power", "Read Power")
        ("trajectory-complete", "Read Trajectory Complete Flag")
        ("voltage", "Read Voltage")

        ("moteus-id", "Moteus ID(s)", cxxopts::value<std::string>()->default_value("1"))
        ("socketcan-iface", "SocketCAN Interface", cxxopts::value<std::string>()->default_value("can0"))
        ("socketcan-ignore-errors", "Ignore SocketCAN errors", cxxopts::value<int>()->default_value("0"))
        ("can-disable-brs", "Disable CAN BRS (bit-rate switching)", cxxopts::value<int>()->default_value("0"))

        ("duration-ms", "Duration in milliseconds", cxxopts::value<int>())

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

    std::unordered_map<std::string, bool> flags;

    auto check_and_add = [&](const std::string& flag, const std::string& key) {
        if (result.count(flag)) {
            flags[key] = true;
        }
    };

    check_and_add("position",            "position");
    check_and_add("velocity",            "velocity");
    check_and_add("torque",              "torque");
    check_and_add("temperature",         "temperature");
    check_and_add("abs-position",        "abs_position");
    check_and_add("aux1-gpio",           "aux1_gpio");
    check_and_add("aux2-gpio",           "aux2_gpio");
    check_and_add("d-current",           "d_current");
    check_and_add("q-current",           "q_current");
    check_and_add("fault",               "fault");
    check_and_add("home-state",          "home_state");
    check_and_add("mode",                "mode");
    check_and_add("motor-temperature",   "motor_temperature");
    check_and_add("power",               "power");
    check_and_add("trajectory-complete", "trajectory_complete");
    check_and_add("voltage",             "voltage");

    if (flags.empty()) {
        flags["position"] = true;
    }

    auto flag_or = [&](const std::string& key) {
        auto it = flags.find(key);
        return it != flags.end() ? it->second : false;
    };

    MoteusAPI::ReadState rs({
        .abs_position        = flag_or("abs_position"),
        .aux1_gpio           = flag_or("aux1_gpio"),
        .aux2_gpio           = flag_or("aux2_gpio"),
        .d_current           = flag_or("d_current"),
        .fault               = flag_or("fault"),
        .home_state          = flag_or("home_state"),
        .mode                = flag_or("mode"),
        .motor_temperature   = flag_or("motor_temperature"),
        .position            = flag_or("position"),
        .power               = flag_or("power"),
        .q_current           = flag_or("q_current"),
        .temperature         = flag_or("temperature"),
        .torque              = flag_or("torque"),
        .trajectory_complete = flag_or("trajectory_complete"),
        .velocity            = flag_or("velocity"),
        .voltage             = flag_or("voltage")
    });

    std::cout << "Sending query..." << std::endl;

    if (result.count("duration-ms")) {
        int duration_ms = result["duration-ms"].as<int>();
        using clock = std::chrono::high_resolution_clock;
        using ms = std::chrono::milliseconds;

        int total_successful_queries = 0;
        ms elapsed(0);

        while (elapsed.count() < duration_ms) {
            for (auto& entry : controllers) {
                auto start_time = clock::now();
                bool ok = entry.mc->read(rs);
                auto end_time = clock::now();

                elapsed += std::chrono::duration_cast<ms>(end_time - start_time);

                if (!ok) {
                    std::cerr << "Read failed for moteus ID " << entry.id << std::endl;
                    return 1;
                }

                ++total_successful_queries;

                if (elapsed.count() >= duration_ms) {
                    break;
                }
            }
        }

        double elapsed_sec = static_cast<double>(elapsed.count()) / 1000.0;
        double freq = (elapsed_sec > 0.0)
            ? static_cast<double>(total_successful_queries) / elapsed_sec
            : 0.0;

        std::cout << "Frequency (successful reads per second across all controllers): "
                  << freq << std::endl;
    }

    bool overall_status = true;
    for (const auto& entry : controllers) {
        bool status = entry.mc->read(rs);
        if (status) {
            std::cout << "\n=== Moteus ID " << entry.id << " ===" << std::endl;
            rs.display();
        } else {
            std::cerr << "Failed to read from moteus ID " << entry.id << std::endl;
            overall_status = false;
        }
    }

    return overall_status ? 0 : 1;
}
