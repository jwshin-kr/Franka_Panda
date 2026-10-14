#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "communication/franka/BridgeTypes.hpp"

namespace communication::franka {

struct RobotConfig {
    // libfranka connects to the robot's control box by IP (FCI).
    std::string robot_ip = "172.16.0.2";
    bool mock_mode = true;
    FrankaControlMode control_mode = FrankaControlMode::kJointImpedance;
    // libfranka torque commands are "without gravity": the robot always adds
    // its own gravity model on top, and that cannot be turned off. When true,
    // the streamed torque is left as-is, so a zero command holds the pose. Set
    // false when gravity comp is supplied externally (e.g. computed from the
    // robot state via Pinocchio in the controller): the bridge then subtracts
    // the robot's model gravity so it is not applied twice.
    bool enable_gravity_comp = true;
    // libfranka's control loop needs a PREEMPT_RT kernel ("enforce"); "ignore"
    // runs anyway (more likely to hit communication_constraints_violation).
    bool enforce_realtime = true;
    // libfranka's own command filtering in Robot::control().
    bool limit_rate = true;
    double cutoff_frequency = 100.0;
    // Robot::setCollisionBehavior: exceeding an upper threshold triggers a
    // reflex (the robot stops and the control loop throws). Values from the
    // acceleration-phase defaults of libfranka's examples_common.cpp.
    std::vector<double> lower_torque_thresholds =
        std::vector<double>(7, 20.0);
    std::vector<double> upper_torque_thresholds =
        std::vector<double>(7, 20.0);
    std::vector<double> lower_force_thresholds =
        std::vector<double>(6, 20.0);
    std::vector<double> upper_force_thresholds =
        std::vector<double>(6, 20.0);
};

struct LoopConfig {
    int loop_hz = 1000;
    double command_timeout_s = 0.05;
};

struct LcmConfig {
    std::string ctrl_channel = "hw_franka_arm_joint_ctrl";
    std::string joint_meas_channel = "hw_franka_arm_joint_meas";
};

struct MappingConfig {
    std::size_t ctrl_joint_offset = 0;
    std::size_t num_arm_joints = 7;
    std::vector<std::string> joint_names;
    std::vector<int> command_sign;
    std::vector<int> measurement_sign;
};

struct SafetyConfig {
    std::vector<double> default_hold_q;
    std::vector<double> hold_kp;
    std::vector<double> hold_kd;
};

struct BridgeConfig {
    std::string name = "franka_panda_arm";
    RobotConfig robot;
    LoopConfig loop;
    LcmConfig lcm;
    MappingConfig mapping;
    SafetyConfig safety;

    static BridgeConfig LoadFromFile(const std::string& path);
};

}  // namespace communication::franka
