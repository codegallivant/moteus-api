#include "moteus/api/moteus_api.hpp"
#include "actuator_interface.hpp"


class MoteusHardwareInterface : ActuatorInterface {
public:
    std::unordered_map<int,MoteusAPI::Controller*> controller_map;
    /// @brief Constructor.
    MoteusHardwareInterface() : ActuatorInterface() {
        
    }

    void init(std::string interface, std::vector<ActuatorSpec> actuators) {
        for(auto it = actuators.begin(); it != actuators.end() ;it++) {
            controller_map[it->id] = MoteusAPI::Controller::create(it->id, interface);
        }
    }

    ~MoteusHardwareInterface() {
        MoteusAPI::Controller::destroyAll();
    }

    void write(std::vector<ActuatorCommand> cmds) {
        for(auto it = cmds.begin();it != cmds.end();it++) {
            MoteusAPI::CommandState cs({
                .position = it->position,
                .velocity = it->velocity,
                .feedforward_torque = it->feedforward_torque,
                .kp_scale = it->kp_scale,
                .kd_scale = it->kd_scale,
                .watchdog_timeout = it->watchdog_timeout,
                .maximum_torque = it->maximum_torque,
                .velocity_limit = it->velocity_limit,
                .stop_position = it->stop_position
            });
            controller_map[it->id]->write(cs);
        }
    }

    std::vector<ActuatorResponse> read(std::string interface, std::vector<ActuatorSpec> actuators) {
        std::vector<ActuatorResponse> results;
        for(auto it = actuators.begin();it != actuators.end();it++) {
            MoteusAPI::ReadState rs({
                .mode = true,
                .fault = true,
                .position = true,
                .velocity = true,
                .torque = true,
                .voltage = true,
                .q_current = true,
                .temperature = true
            });
            controller_map[it->id]->read(rs);
            ActuatorResponse result = {
                .id = static_cast<uint32_t>(it->id),
                .mode = static_cast<uint8_t>(rs.mode.value.value()),
                .fault = static_cast<int8_t>(rs.fault.value.value()),
                .position = rs.position.value.value(),
                .velocity = rs.velocity.value.value(),
                .torque = rs.torque.value.value(),
                .voltage = rs.voltage.value.value(),
                .current = rs.q_current.value.value(),
                .temperature = rs.temperature.value.value()
            };
            results.push_back(result); 
        }
        return results;
    }

    void stop() {
        for(auto it = controller_map.begin() ; it != controller_map.end() ; it++) {
            it->second->terminate();
        }
    }

    void set_zero(uint32_t id) {
        
    }
};
