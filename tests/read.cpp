#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <csignal>
#include <sstream>
#include <chrono>

#include "api/moteus_driver_controller.hpp"
#include "lib/cxxopts.hpp"

void signalHandler(int signum) {
    MoteusDriverController::destroyAll();
}

int main(int argc, char** argv) {

    std::signal(SIGINT, signalHandler);

    cxxopts::Options options("MoteusRead", "Read Moteus Controller State");
    options.allow_unrecognised_options();

    options.add_options()
        // Reading options
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

        // transport args
        ("moteus-id", "Moteus ID(s)", cxxopts::value<std::string>()->default_value("1"))
        ("socketcan-iface", "SocketCAN Interface", cxxopts::value<std::string>()->default_value("can0"))
        ("socketcan-ignore-errors", "Ignore SocketCAN errors", cxxopts::value<int>()->default_value("0"))
        ("can-disable-brs", "Disable CAN BRS (bit-rate switching)", cxxopts::value<int>()->default_value("0"))

        // Timing (Keep as int)
        ("duration-ms", "Duration in milliseconds", cxxopts::value<int>())

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

    MoteusDriverController::motor_state readState;

    auto check_and_add = [&](const std::string& flag, const std::string& key) {
        if (result.count(flag)) {
            readState[key] = std::nullopt;
        }
    };

    check_and_add("position",           "position");
    check_and_add("velocity",           "velocity");
    check_and_add("torque",             "torque");
    check_and_add("temperature",        "temperature");
    check_and_add("abs-position",       "abs_position");
    check_and_add("aux1-gpio",          "aux1_gpio");
    check_and_add("aux2-gpio",          "aux2_gpio");
    check_and_add("d-current",          "d_current");
    check_and_add("q-current",          "q_current");
    check_and_add("fault",              "fault");
    check_and_add("home-state",         "home_state");
    check_and_add("mode",               "mode");
    check_and_add("motor-temperature",  "motor_temperature");
    check_and_add("power",              "power");
    check_and_add("trajectory-complete","trajectory_complete");
    check_and_add("voltage",            "voltage");

    if (readState.empty()) {
        readState["position"] = std::nullopt;
    }

    std::cout << "Sending query..." << std::endl;

    // auto q_com = controllers.front().mc->getReadFormat(readState);
    if (result.count("duration-ms")) {
        auto duration_ms = result["duration-ms"].as<int>();
        int total_successful_queries = 0;
        int elapsed_ms = 0;
        while (true) {
            if (elapsed_ms >= duration_ms) {
                break;
            }
            for (auto& entry : controllers) {
                auto start_time = std::chrono::high_resolution_clock::now();
                auto result_q = entry.mc->read(readState); 
                // const auto result_q = entry.mc->controller->SetQuery(&q_com);
                auto end_time = std::chrono::high_resolution_clock::now();
                int this_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                elapsed_ms += this_duration;

                if (!result_q) {
                    std::cerr << "Read failed for moteus ID " << entry.id << std::endl;
                    return 1;
                } else {
                    total_successful_queries++;
                }
            }
        }

        double elapsed_sec = static_cast<double>(elapsed_ms) / 1000.0;

        std::cout << "Frequency (successful reads per second across all controllers): "
                  << (elapsed_sec > 0.0 ? total_successful_queries / elapsed_sec : 0.0)
                  << std::endl;
    }

    // Final read from all controllers one after the other
    bool overall_status = true;
    for (const auto& entry : controllers) {
        bool status = entry.mc->read(readState);
        if (status) {
            std::cout << "\n=== Moteus ID " << entry.id << " ===" << std::endl;
            MoteusDriverController::displayState(readState);
        } else {
            std::cerr << "Failed to read from moteus ID " << entry.id << std::endl;
            overall_status = false;
        }
    }

    return overall_status ? 0 : 1;
}
