#include "communication/franka/FrankaClient.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>

#include <franka/exception.h>

namespace communication::franka {

namespace {

// The Panda arm always has 7 joints (franka::RobotState uses std::array<7>).
constexpr std::size_t kFrankaArmJoints = 7;

double NowSeconds()
{
    // Wall-clock to match the controller's Python time.time() timestamps,
    // which downstream consumers compare against.
    using clock = std::chrono::system_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
}

template <std::size_t N>
std::array<double, N> ToArray(const std::vector<double>& values)
{
    std::array<double, N> out{};
    std::copy_n(values.begin(), N, out.begin());
    return out;
}

}  // namespace

FrankaClient::~FrankaClient()
{
    finish_requested_.store(true);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

void FrankaClient::Initialize(const BridgeConfig& config)
{
    mock_mode_ = config.robot.mock_mode;
    enable_gravity_comp_ = config.robot.enable_gravity_comp;
    limit_rate_ = config.robot.limit_rate;
    cutoff_frequency_ = config.robot.cutoff_frequency;
    command_timeout_s_ = config.loop.command_timeout_s;
    num_arm_joints_ = config.mapping.num_arm_joints;
    initialized_ = true;

    last_state_.timestamp = NowSeconds();
    last_state_.q = config.safety.default_hold_q;
    last_state_.qd.assign(num_arm_joints_, 0.0);
    last_state_.tau.assign(num_arm_joints_, 0.0);

    if (mock_mode_) {
        return;
    }

    if (config.robot.robot_ip.empty() || config.robot.robot_ip == "REPLACE_ME") {
        throw std::runtime_error(
            "robot.robot_ip must be set in the bridge config when mock_mode is false.");
    }

    std::cout << "[franka_client] Connecting to robot at '" << config.robot.robot_ip
              << "' via libfranka..." << std::endl;
    robot_ = std::make_unique<::franka::Robot>(
        config.robot.robot_ip,
        config.robot.enforce_realtime ? ::franka::RealtimeConfig::kEnforce
                                      : ::franka::RealtimeConfig::kIgnore);

    auto state = robot_->readOnce();
    if (state.robot_mode == ::franka::RobotMode::kUserStopped) {
        throw std::runtime_error(
            "Franka robot is user-stopped; release the external activation device.");
    }
    if (state.robot_mode == ::franka::RobotMode::kReflex ||
        static_cast<bool>(state.current_errors)) {
        std::cout << "[franka_client] Robot in error/reflex state, attempting to recover..."
                  << std::endl;
        try {
            robot_->automaticErrorRecovery();
        } catch (const ::franka::Exception& ex) {
            throw std::runtime_error(
                std::string("Failed to recover Franka robot from error: ") + ex.what());
        }
        std::cout << "[franka_client] Errors cleared." << std::endl;
        state = robot_->readOnce();
    }

    // libfranka has no Enable(): the joints are unlocked and FCI is activated
    // in Desk, after which the robot reports Idle.
    if (state.robot_mode != ::franka::RobotMode::kIdle) {
        throw std::runtime_error(
            "Franka robot is not Idle (robot_mode=" +
            std::to_string(static_cast<int>(state.robot_mode)) +
            "); unlock the joints and activate FCI in Desk.");
    }
    std::cout << "[franka_client] Robot is Idle (operational)." << std::endl;

    if (num_arm_joints_ != kFrankaArmJoints) {
        throw std::runtime_error(
            "Franka robot has " + std::to_string(kFrankaArmJoints) +
            " joints but config expects " + std::to_string(num_arm_joints_) + ".");
    }
    latest_state_ = ToArmState(state);

    // Only joint torque control is wired up; we compute the full impedance law
    // (kp*(q_des-q) + kd*(qd_des-qd) + tau_ff) client-side from joint_ctrl_t
    // and stream it from libfranka's control callback so kp/kd can change per
    // command.
    if (config.robot.control_mode != FrankaControlMode::kJointTorque) {
        throw std::runtime_error(
            "Only joint_torque is implemented for real-mode streaming; "
            "set robot.control_mode to 'joint_torque'.");
    }

    // The robot's own model, used to take its built-in gravity back out when
    // the controller supplies gravity (enable_gravity_comp: false).
    model_ = std::make_unique<::franka::Model>(robot_->loadModel());

    robot_->setCollisionBehavior(
        ToArray<7>(config.robot.lower_torque_thresholds),
        ToArray<7>(config.robot.upper_torque_thresholds),
        ToArray<6>(config.robot.lower_force_thresholds),
        ToArray<6>(config.robot.upper_force_thresholds));

    // Stay out of torque control until the bridge sees a fresh command; the
    // robot's own controller holds the pose meanwhile (Idle). EnsureActiveMode()
    // starts the control loop on the first fresh command and EnsureIdleMode()
    // ends it when commands go stale.
    std::cout << "[franka_client] Idle (waiting for commands)." << std::endl;
    is_active_mode_ = false;
}

void FrankaClient::EnsureActiveMode()
{
    if (mock_mode_ || robot_ == nullptr) {
        is_active_mode_ = true;
        return;
    }
    RethrowControlError();
    if (is_active_mode_ && control_running_.load()) {
        return;
    }
    // The previous loop may have ended on its own (stale command seen inside
    // the callback); reap its thread before starting a new one.
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    std::cout << "[franka_client] Starting torque control loop." << std::endl;
    {
        // Drop the command from the previous activation: it is stale and would
        // end the new loop on its first tick.
        std::lock_guard<std::mutex> lock(mutex_);
        latest_command_.reset();
    }
    finish_requested_.store(false);
    control_running_.store(true);
    control_thread_ = std::thread(&FrankaClient::ControlLoop, this);
    is_active_mode_ = true;
}

void FrankaClient::EnsureIdleMode()
{
    if (mock_mode_ || robot_ == nullptr || !is_active_mode_) {
        is_active_mode_ = false;
        return;
    }
    std::cout << "[franka_client] Ending torque control loop (commands stale)."
              << std::endl;
    finish_requested_.store(true);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    is_active_mode_ = false;
    RethrowControlError();
}

void FrankaClient::StartRealtime()
{
    if (!initialized_) {
        throw std::runtime_error("FrankaClient must be initialized before start.");
    }
    realtime_started_ = true;
}

void FrankaClient::Stop()
{
    realtime_started_ = false;
    finish_requested_.store(true);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    is_active_mode_ = false;
    if (robot_ != nullptr) {
        try {
            robot_->stop();
        } catch (const ::franka::Exception& ex) {
            std::cerr << "[franka_client] stop() failed: " << ex.what() << std::endl;
        }
    }
}

void FrankaClient::StreamCommand(const ArmCommand& command)
{
    if (!realtime_started_) {
        throw std::runtime_error("FrankaClient realtime loop has not started.");
    }

    if (mock_mode_) {
        last_state_.timestamp = command.timestamp;
        last_state_.q = command.q_des;
        last_state_.qd = command.qd_des;
        last_state_.tau = command.tau_ff;
        return;
    }

    if (robot_ == nullptr) {
        throw std::runtime_error(
            "FrankaClient real-mode StreamCommand called with no robot handle.");
    }

    if (command.q_des.size() != num_arm_joints_ ||
        command.qd_des.size() != num_arm_joints_ ||
        command.tau_ff.size() != num_arm_joints_ ||
        command.kp.size() != num_arm_joints_ ||
        command.kd.size() != num_arm_joints_) {
        throw std::runtime_error(
            "ArmCommand vectors must all have num_arm_joints entries.");
    }

    // The torque itself is computed in ControlCallback with the robot state of
    // that 1 kHz tick; here we only hand over the latest command.
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = command;
}

ArmState FrankaClient::ReadState()
{
    if (mock_mode_ || robot_ == nullptr) {
        return last_state_;
    }
    RethrowControlError();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!control_running_.load()) {
        // readOnce() is only allowed while no control loop is running; during
        // control the callback keeps latest_state_ up to date instead.
        latest_state_ = ToArmState(robot_->readOnce());
    }
    return latest_state_;
}

void FrankaClient::ControlLoop()
{
    try {
        robot_->control(
            [this](const ::franka::RobotState& state, ::franka::Duration /*period*/) {
                return ControlCallback(state);
            },
            limit_rate_,
            cutoff_frequency_);
    } catch (const std::exception& ex) {
        std::cerr << "[franka_client] Control loop aborted: " << ex.what() << std::endl;
        std::lock_guard<std::mutex> lock(mutex_);
        control_error_ = ex.what();
    }
    control_running_.store(false);
}

::franka::Torques FrankaClient::ControlCallback(const ::franka::RobotState& state)
{
    const std::array<double, kFrankaArmJoints> zeros{};

    std::optional<ArmCommand> command;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_state_ = ToArmState(state);
        command = latest_command_;
    }

    if (finish_requested_.load()) {
        return ::franka::MotionFinished(::franka::Torques(zeros));
    }
    // No command handed over yet (first ticks after the loop starts): zero
    // torque, i.e. only the robot's own gravity compensation.
    if (!command.has_value()) {
        return ::franka::Torques(zeros);
    }
    // Same freshness rule as the bridge's SafetyMonitor, checked here as well
    // because this callback keeps running even if the bridge loop stalls.
    if (NowSeconds() - command->timestamp > command_timeout_s_) {
        return ::franka::MotionFinished(::franka::Torques(zeros));
    }

    // Link-side q and dq of this tick, as in libfranka's own impedance examples.
    std::array<double, kFrankaArmJoints> torque{};
    for (std::size_t i = 0; i < kFrankaArmJoints; ++i) {
        torque[i] = command->kp[i] * (command->q_des[i] - state.q[i])
                    + command->kd[i] * (command->qd_des[i] - state.dq[i])
                    + command->tau_ff[i];
    }

    // enable_gravity_comp (config: robot.enable_gravity_comp): the robot always
    // adds its own model gravity on top of the commanded torque. When false
    // (gravity comp supplied externally, e.g. Pinocchio in the controller), the
    // command above already includes gravity, so the robot's is subtracted to
    // keep it from being applied twice.
    if (!enable_gravity_comp_) {
        const auto gravity = model_->gravity(state);
        for (std::size_t i = 0; i < kFrankaArmJoints; ++i) {
            torque[i] -= gravity[i];
        }
    }
    return ::franka::Torques(torque);
}

void FrankaClient::RethrowControlError() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!control_error_.empty()) {
        throw std::runtime_error("Franka control loop aborted: " + control_error_);
    }
}

ArmState FrankaClient::ToArmState(const ::franka::RobotState& state) const
{
    ArmState out;
    // Wall-clock, not state.time (time since the robot booted), so it lines up
    // with the controller's time.time() timestamps.
    out.timestamp = NowSeconds();
    out.q.assign(state.q.begin(), state.q.begin() + num_arm_joints_);
    out.qd.assign(state.dq.begin(), state.dq.begin() + num_arm_joints_);
    out.tau.assign(state.tau_J.begin(), state.tau_J.begin() + num_arm_joints_);
    return out;
}

}  // namespace communication::franka
