#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <franka/model.h>
#include <franka/robot.h>

#include "communication/franka/BridgeConfig.hpp"
#include "communication/franka/BridgeTypes.hpp"

namespace communication::franka {

class FrankaClient {
public:
    FrankaClient() = default;
    ~FrankaClient();

    void Initialize(const BridgeConfig& config);
    void StartRealtime();
    void Stop();
    void StreamCommand(const ArmCommand& command);
    [[nodiscard]] ArmState ReadState();

    // Start / end libfranka's torque control loop. Idempotent: a no-op if the
    // loop is already in the requested state. Bridge calls EnsureActiveMode
    // when a fresh command arrives and EnsureIdleMode when commands go stale,
    // so the robot's own controller (Idle) holds the current pose.
    void EnsureActiveMode();
    void EnsureIdleMode();

private:
    // Robot::control() blocks for as long as the loop runs, so it gets its
    // own thread; StreamCommand only hands the latest command to it.
    void ControlLoop();
    ::franka::Torques ControlCallback(const ::franka::RobotState& state);
    // A control-loop exception (reflex, user stop, network) ends the run: it is
    // rethrown on the bridge thread so the bridge exits.
    void RethrowControlError() const;
    ArmState ToArmState(const ::franka::RobotState& state) const;

    bool mock_mode_ = true;
    bool enable_gravity_comp_ = true;
    bool limit_rate_ = true;
    double cutoff_frequency_ = 100.0;
    double command_timeout_s_ = 0.05;
    bool initialized_ = false;
    bool realtime_started_ = false;
    bool is_active_mode_ = false;
    std::size_t num_arm_joints_ = 0;
    ArmState last_state_;
    std::unique_ptr<::franka::Robot> robot_;
    std::unique_ptr<::franka::Model> model_;

    std::thread control_thread_;
    std::atomic<bool> control_running_{false};
    std::atomic<bool> finish_requested_{false};
    mutable std::mutex mutex_;  // guards latest_command_, latest_state_, control_error_
    std::optional<ArmCommand> latest_command_;
    ArmState latest_state_;
    std::string control_error_;
};

}  // namespace communication::franka
