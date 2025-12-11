#include "moteus/api/moteus_api.hpp"

namespace MoteusAPI {

// ======================
// CommandState
// ======================
CommandState::CommandState(const CommandStateParams& p)
    : accel_limit(p.accel_limit),
      feedforward_torque(p.feedforward_torque),
      ilimit_scale(p.ilimit_scale),
      kp_scale(p.kp_scale),
      kd_scale(p.kd_scale),
      maximum_torque(p.maximum_torque),
      position(p.position),
      stop_position(p.stop_position),
      watchdog_timeout(p.watchdog_timeout),
      velocity(p.velocity),
      velocity_limit(p.velocity_limit)
{
    createMaps();
    calculateFormat();
    calculateCommand();
}

std::unordered_map<std::string, const std::optional<double>*> 
CommandState::getValues() const {
    return valueMap;
}

void CommandState::display() const {
    displayPtrMap<const std::optional<double>>(valueMap);
}

void CommandState::createMaps() {
    valueMap = {
        {"accel_limit", &accel_limit},
        {"feedforward_torque", &feedforward_torque},
        {"ilimit_scale", &ilimit_scale},
        {"kp_scale", &kp_scale},
        {"kd_scale", &kd_scale},
        {"maximum_torque", &maximum_torque},
        {"position", &position},
        {"stop_position", &stop_position},
        {"watchdog_timeout", &watchdog_timeout},
        {"velocity", &velocity},
        {"velocity_limit", &velocity_limit},
    };

    formatMap = {
        {"accel_limit", &format.accel_limit},
        {"feedforward_torque", &format.feedforward_torque},
        {"ilimit_scale", &format.ilimit_scale},
        {"kp_scale", &format.kp_scale},
        {"kd_scale", &format.kd_scale},
        {"maximum_torque", &format.maximum_torque},
        {"position", &format.position},
        {"stop_position", &format.stop_position},
        {"watchdog_timeout", &format.watchdog_timeout},
        {"velocity", &format.velocity},
        {"velocity_limit", &format.velocity_limit},
    };

    commandMap = {
        {"accel_limit", &command.accel_limit},
        {"feedforward_torque", &command.feedforward_torque},
        {"ilimit_scale", &command.ilimit_scale},
        {"kp_scale", &command.kp_scale},
        {"kd_scale", &command.kd_scale},
        {"maximum_torque", &command.maximum_torque},
        {"position", &command.position},
        {"stop_position", &command.stop_position},
        {"watchdog_timeout", &command.watchdog_timeout},
        {"velocity", &command.velocity},
        {"velocity_limit", &command.velocity_limit},
    };
}

void CommandState::calculateFormat() {
    for (auto& [key, resPtr] : formatMap) {
        auto value = *valueMap[key];
        *resPtr = value.has_value()
            ? mjbots::moteus::Resolution::kFloat
            : mjbots::moteus::Resolution::kIgnore;
    }
}

void CommandState::calculateCommand() {
    for (auto& [key, cmdPtr] : commandMap) {
        auto value = *valueMap[key];
        if (value.has_value()) {
            *cmdPtr = value.value();
        }
    }
}

// ======================
// ReadState
// ======================
ReadState::ReadState(const ReadStateParams& p)
    : abs_position(p.abs_position),
      aux1_gpio(p.aux1_gpio),
      aux2_gpio(p.aux2_gpio),
      d_current(p.d_current),
      fault(p.fault),
      home_state(p.home_state),
      mode(p.mode),
      motor_temperature(p.motor_temperature),
      position(p.position),
      power(p.power),
      q_current(p.q_current),
      temperature(p.temperature),
      torque(p.torque),
      trajectory_complete(p.trajectory_complete),
      velocity(p.velocity),
      voltage(p.voltage)
{
    createMaps();
    calculateFormat();
}

std::unordered_map<std::string, std::optional<double>*> 
ReadState::getValues() {
    return valueMap;
}

void ReadState::display() {
    displayPtrMap<std::optional<double>>(valueMap);
}

void ReadState::populateState(mjbots::moteus::Query::Result values) {
    std::unordered_map<std::string, std::optional<double>> resultMap = {
        {"abs_position", values.abs_position},
        {"aux1_gpio", values.aux1_gpio},
        {"aux2_gpio", values.aux2_gpio},
        {"d_current", values.d_current},
        {"fault", values.fault},
        {"home_state", static_cast<float>(values.home_state)},
        {"mode", static_cast<float>(values.mode)},
        {"motor_temperature", values.motor_temperature},
        {"position", values.position},
        {"power", values.power},
        {"q_current", values.q_current},
        {"temperature", values.temperature},
        {"torque", values.torque},
        {"trajectory_complete", values.trajectory_complete},
        {"velocity", values.velocity},
        {"voltage", values.voltage}
    };

    for (auto& [key, opt] : resultMap) {
        if (paramMap[key]->flag) {
            paramMap[key]->value = opt;
        }
    }
}

void ReadState::createMaps() {
    paramMap = {
        {"abs_position", &abs_position},
        {"aux1_gpio", &aux1_gpio},
        {"aux2_gpio", &aux2_gpio},
        {"d_current", &d_current},
        {"fault", &fault},
        {"home_state", &home_state},
        {"mode", &mode},
        {"motor_temperature", &motor_temperature},
        {"position", &position},
        {"power", &power},
        {"q_current", &q_current},
        {"temperature", &temperature},
        {"torque", &torque},
        {"trajectory_complete", &trajectory_complete},
        {"velocity", &velocity},
        {"voltage", &voltage},
    };

    valueMap = {
        {"abs_position", &abs_position.value},
        {"aux1_gpio", &aux1_gpio.value},
        {"aux2_gpio", &aux2_gpio.value},
        {"d_current", &d_current.value},
        {"fault", &fault.value},
        {"home_state", &home_state.value},
        {"mode", &mode.value},
        {"motor_temperature", &motor_temperature.value},
        {"position", &position.value},
        {"power", &power.value},
        {"q_current", &q_current.value},
        {"temperature", &temperature.value},
        {"torque", &torque.value},
        {"trajectory_complete", &trajectory_complete.value},
        {"velocity", &velocity.value},
        {"voltage", &voltage.value},
    };

    formatMap = {
        {"abs_position", &format.abs_position},
        {"aux1_gpio", &format.aux1_gpio},
        {"aux2_gpio", &format.aux2_gpio},
        {"d_current", &format.d_current},
        {"fault", &format.fault},
        {"home_state", &format.home_state},
        {"mode", &format.mode},
        {"motor_temperature", &format.motor_temperature},
        {"position", &format.position},
        {"power", &format.power},
        {"q_current", &format.q_current},
        {"temperature", &format.temperature},
        {"torque", &format.torque},
        {"trajectory_complete", &format.trajectory_complete},
        {"velocity", &format.velocity},
        {"voltage", &format.voltage},
    };

    formatDefaultsMap = {
        {"abs_position", mjbots::moteus::Resolution::kFloat},
        {"aux1_gpio", mjbots::moteus::Resolution::kFloat},
        {"aux2_gpio", mjbots::moteus::Resolution::kFloat},
        {"d_current", mjbots::moteus::Resolution::kFloat},
        {"fault", mjbots::moteus::Resolution::kInt8},
        {"home_state", mjbots::moteus::Resolution::kInt16},
        {"mode", mjbots::moteus::Resolution::kInt8},
        {"motor_temperature", mjbots::moteus::Resolution::kFloat},
        {"position", mjbots::moteus::Resolution::kFloat},
        {"power", mjbots::moteus::Resolution::kFloat},
        {"q_current", mjbots::moteus::Resolution::kFloat},
        {"temperature", mjbots::moteus::Resolution::kFloat},
        {"torque", mjbots::moteus::Resolution::kFloat},
        {"trajectory_complete", mjbots::moteus::Resolution::kInt16},
        {"velocity", mjbots::moteus::Resolution::kFloat},
        {"voltage", mjbots::moteus::Resolution::kFloat},
    };
}

void ReadState::calculateFormat() {
    for (auto& [key, resPtr] : formatMap) {
        Param& param = *paramMap[key];
        *resPtr = param.flag ? formatDefaultsMap[key]
                             : mjbots::moteus::Resolution::kIgnore;
    }
}

// ======================
// Controller
// ======================
Controller::Controller(
    int moteus_id,
    std::string socketcan_iface,
    int socketcan_ignore_errors,
    int socketcan_disable_brs)
{
    mjbots::moteus::Controller::Options options;
    options.id = moteus_id;
    options.default_query = false;

    std::vector<std::string> transport_args = {
        "--socketcan-iface", socketcan_iface,
        "--socketcan-ignore-errors", std::to_string(socketcan_ignore_errors),
        "--can-disable-brs", std::to_string(socketcan_disable_brs),
    };
    auto transport = mjbots::moteus::Controller::MakeSingletonTransport(transport_args);
    options.transport = transport;

    internal_controller = std::make_unique<mjbots::moteus::Controller>(options);
    internal_controller->SetStop();
}

Controller::~Controller() {
    terminate();
}

void Controller::terminate() {
    internal_controller->SetStop();
}

MoteusAPI::Controller* Controller::create(
    int moteus_id,
    std::string socketcan_iface,
    int socketcan_ignore_errors,
    int socketcan_disable_brs)
{
    registry.push_back(std::make_unique<MoteusAPI::Controller>(
        moteus_id,
        socketcan_iface,
        socketcan_ignore_errors,
        socketcan_disable_brs));
    return registry.back().get();
}

void Controller::destroyAll() {
    std::cout << "Resetting all Moteus Driver Controllers\n";
    registry.clear();
}

bool Controller::read(ReadState& rs) {
    const auto result = internal_controller->SetQuery(&rs.format);
    if (!result) return false;
    rs.populateState(result->values);
    return true;
}

bool Controller::write(const CommandState& cs, ReadState& rs) {
    auto result = internal_controller->SetPosition(cs.command, &cs.format, &rs.format);
    if (!result) return false;
    rs.populateState(result->values);
    return true;
}

void Controller::write(const CommandState& cs) {
    internal_controller->SetPosition(cs.command, &cs.format);
}

void Controller::writeDuration(const CommandState& cs, int duration_ms) {
    auto start_time = std::chrono::high_resolution_clock::now();
    int count = 0;
    int elapsed = 0;
    while (elapsed < duration_ms) {
        auto t0 = std::chrono::high_resolution_clock::now();
        write(cs);
        auto t1 = std::chrono::high_resolution_clock::now();
        count++;
        elapsed += std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    }
    std::cout << "Control Frequency (writes/s): "
              << count / (elapsed / 1000.0) << std::endl;
}

} // namespace MoteusAPI
