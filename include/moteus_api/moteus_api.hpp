#pragma once

#include "moteus.h"
#include "moteus_api.hpp"

#include <unordered_map>
#include <memory>
#include <iostream>
#include <string>
#include <optional>
#include <vector>
#include <chrono>

namespace MoteusAPI {

    // -------------------------------------------------------------------------
    // Parameter structs
    // -------------------------------------------------------------------------

    struct CommandStateParams {
        std::optional<double> accel_limit = std::nullopt;
        std::optional<double> feedforward_torque = std::nullopt;
        std::optional<double> ilimit_scale = std::nullopt;
        std::optional<double> kp_scale = std::nullopt;
        std::optional<double> kd_scale = std::nullopt;
        std::optional<double> maximum_torque = std::nullopt;
        std::optional<double> position = std::nullopt;
        std::optional<double> stop_position = std::nullopt;
        std::optional<double> watchdog_timeout = std::nullopt;
        std::optional<double> velocity = std::nullopt;
        std::optional<double> velocity_limit = std::nullopt;
    };

    struct ReadStateParams {
        bool abs_position = false;
        bool aux1_gpio = false;
        bool aux2_gpio = false;
        bool d_current = false;
        bool fault = false;
        bool home_state = false;
        bool mode = false;
        bool motor_temperature = false;
        bool position = true;
        bool power = false;
        bool q_current = false;
        bool temperature = false;
        bool torque = false;
        bool trajectory_complete = false;
        bool velocity = false;
        bool voltage = false;
    };

    typedef struct ViolationStats {
        double current;
        bool motor_current_condition;
        bool moteus_current_condition;
        bool d_current_condition;
        bool fault_condition;
        bool result;
    } ViolationStats;

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    template<typename T>
    void displayPtrMap(std::unordered_map<std::string, T*> m) {
        for (const auto& [key, opt_val] : m) {
            if (opt_val && opt_val->has_value()) {
                std::cout << key << ": ";
                std::cout << opt_val->value();
                std::cout << "\n";
            } 
            // else {
            //     std::cout << "null";
            // }
        }
    }

    // -------------------------------------------------------------------------
    // CommandState
    // -------------------------------------------------------------------------

    class CommandState {
    public:
        // Values
        const std::optional<double> accel_limit;    
        const std::optional<double> feedforward_torque;
        const std::optional<double> ilimit_scale;
        const std::optional<double> kp_scale;
        const std::optional<double> kd_scale;
        const std::optional<double> maximum_torque;
        const std::optional<double> position;
        const std::optional<double> stop_position;
        const std::optional<double> watchdog_timeout;
        const std::optional<double> velocity;
        const std::optional<double> velocity_limit;
        
        // Maps (value view)
        std::unordered_map<std::string, const std::optional<double>*> valueMap;

        // Moteus Driver Resolution/Format
        mjbots::moteus::PositionMode::Format format{}; // command format
        mjbots::moteus::PositionMode::Command command{};

        explicit CommandState(const CommandStateParams& p);

        std::unordered_map<std::string, const std::optional<double>*> getValues() const;

        void display() const;

    private:
        // Maps used internally
        std::unordered_map<std::string, mjbots::moteus::Resolution*> formatMap;
        std::unordered_map<std::string, double*> commandMap;

        void createMaps();
        void calculateFormat();
        void calculateCommand();
    };

    // -------------------------------------------------------------------------
    // ReadState
    // -------------------------------------------------------------------------

    class ReadState {
    public:
        struct Param {
            std::optional<double> value; // Result value
            const bool flag;             // Request flag
            explicit Param(bool enabled = false) : flag(enabled) {}
        };

        // Parameters
        Param abs_position;
        Param aux1_gpio;
        Param aux2_gpio;
        Param d_current;
        Param fault;               // int
        Param home_state;          // int
        Param mode;                // int
        Param motor_temperature;
        Param position;
        Param power;
        Param q_current;
        Param temperature;
        Param torque;
        Param trajectory_complete; // int
        Param velocity;
        Param voltage;

        // Maps
        std::unordered_map<std::string, std::optional<double>*> valueMap;
        std::unordered_map<std::string, Param*> paramMap;

        // Moteus Driver Resolution/Format
        mjbots::moteus::Query::Format format{};

        explicit ReadState(const ReadStateParams& p);

        std::unordered_map<std::string, std::optional<double>*> getValues();

        void display();

        void populateState(mjbots::moteus::Query::Result values);

    private:
        // Maps for format
        std::unordered_map<std::string, mjbots::moteus::Resolution*> formatMap;
        std::unordered_map<std::string, mjbots::moteus::Resolution> formatDefaultsMap;

        void createMaps();
        void calculateFormat();
    };

    // -------------------------------------------------------------------------
    // Controller
    // -------------------------------------------------------------------------

    class Controller {
    public:
        inline static std::vector<std::unique_ptr<MoteusAPI::Controller>> registry{};

        std::unique_ptr<mjbots::moteus::Controller> internal_controller;

        double max_moteus_current = 0;
        double max_motor_current = 0;
        double max_d_current = 0;
        int violation_limit_ms = 0;
        
        explicit Controller(
            int moteus_id = 1,
            std::string socketcan_iface = "can0",
            int socketcan_ignore_errors = 0,
            int socketcan_disable_brs = 0
        );

        ~Controller();

        void terminate();

        // Centralized creation (tracked by registry)
        static MoteusAPI::Controller* create(
            int moteus_id = 1,
            std::string socketcan_iface = "can0",
            int socketcan_ignore_errors = 0,
            int socketcan_disable_brs = 0
        );

        // Destroy all controllers created via create()
        static void destroyAll();

        bool read(ReadState& rs);

        bool write(const CommandState& cs, ReadState& rs);

        void write(const CommandState& cs);

        void writeDuration(const CommandState& cs, int duration_ms, bool safety = true, bool display = false, int interval_ms = 10);

        std::string diagnosticCommand(std::string);

        bool setZeroOffset();

        void configureSafety(double max_motor_current, double max_moteus_current = 12, double max_d_current = 5, double violation_limit_ms = 100);

        ViolationStats checkSafety(ReadState& rs);
    };

} // namespace MoteusAPI
