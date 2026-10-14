#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace communication::franka {

enum class FrankaControlMode {
    kJointImpedance,
    kJointTorque,
};

inline std::string ToString(FrankaControlMode mode)
{
    switch (mode) {
    case FrankaControlMode::kJointImpedance:
        return "joint_impedance";
    case FrankaControlMode::kJointTorque:
        return "joint_torque";
    }
    throw std::runtime_error("Unhandled FrankaControlMode.");
}

inline FrankaControlMode ParseControlMode(const std::string& mode)
{
    if (mode == "joint_impedance") {
        return FrankaControlMode::kJointImpedance;
    }
    if (mode == "joint_torque") {
        return FrankaControlMode::kJointTorque;
    }
    throw std::runtime_error("Unsupported Franka control mode: " + mode);
}

struct ArmCommand {
    double timestamp = 0.0;
    std::vector<double> q_des;
    std::vector<double> qd_des;
    std::vector<double> tau_ff;
    std::vector<double> kp;
    std::vector<double> kd;
    bool valid = false;

    [[nodiscard]] std::size_t size() const { return q_des.size(); }
};

struct ArmState {
    double timestamp = 0.0;
    std::vector<double> q;
    std::vector<double> qd;
    std::vector<double> tau;

    [[nodiscard]] std::size_t size() const { return q.size(); }
};

}  // namespace communication::franka
