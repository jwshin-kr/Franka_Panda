#pragma once

#include <optional>

#include "communication/franka/BridgeConfig.hpp"
#include "communication/franka/BridgeTypes.hpp"

namespace communication::franka {

class SafetyMonitor {
public:
    explicit SafetyMonitor(const BridgeConfig& config);

    // Returns true iff [command] is present and its timestamp is no older
    // than command_timeout_s relative to [now_s]. The bridge uses this to
    // decide whether to stream commands or to end the torque control loop so
    // the robot's own controller (Idle) holds the current pose.
    [[nodiscard]] bool IsCommandFresh(
        const std::optional<ArmCommand>& command,
        double now_s) const;

private:
    double command_timeout_s_ = 0.05;
};

}  // namespace communication::franka
