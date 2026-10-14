// Dump libfranka's own model gravity g(q) across a joint-space sweep, so it
// can be compared against the URDF/Pinocchio gravity used by our controller.
// Model::gravity(q, m_total, F_x_Ctotal) accepts ANY configuration, so this
// needs the robot connected but NOT moving - it's a pure model-vs-model
// comparison with no friction or torque-sensor confound.
//
//   Usage: franka_dynamics_dump <robot_ip> [out.csv]
//
// Writes a CSV with columns j0..j6,g0..g6 (rad, Nm). Compare with Pinocchio's
// computeGeneralizedGravity on assets/scene/franka_panda/urdf/panda_viser.urdf
// (arm joints; fingers at any value).
//
// IMPORTANT: the Franka bridge (or any other libfranka Robot client) must NOT
// be running - only one FCI connection to the robot is allowed at a time. This
// tool never starts a control loop and never streams torque; it only reads the
// state and evaluates the model.

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {

constexpr int kNumJoints = 7;

void WriteRow(
    std::ostream& os,
    const std::array<double, kNumJoints>& q,
    const std::array<double, kNumJoints>& g)
{
    for (int i = 0; i < kNumJoints; ++i) os << q[i] << ",";
    for (int i = 0; i < kNumJoints; ++i) {
        os << g[i];
        os << (i == kNumJoints - 1 ? '\n' : ',');
    }
}

// A representative set of poses to probe. Sweeps joint2 and joint4 (the main
// gravity-loaded joints) through the Panda's range with the others at zero, so
// the per-pose g(q) traces out the gravity curve for those joints. joint4 of
// the Panda only moves in [-176, -4] deg, so it is swept on that side.
std::vector<std::array<double, kNumJoints>> BuildPoses()
{
    std::vector<std::array<double, kNumJoints>> poses;
    const double deg = M_PI / 180.0;
    for (int a = -90; a <= 90; a += 15) {  // joint2 (index 1)
        std::array<double, kNumJoints> q{};
        q[1] = a * deg;
        poses.push_back(q);
    }
    for (int a = -165; a <= -15; a += 15) {  // joint4 (index 3)
        std::array<double, kNumJoints> q{};
        q[3] = a * deg;
        poses.push_back(q);
    }
    // The Franka "ready" pose and this cell's default pose (see
    // configs/franka_panda/env/default.yaml).
    poses.push_back({0.0, -45 * deg, 0.0, -135 * deg, 0.0, 90 * deg, 45 * deg});
    poses.push_back({0.0, 0.0, 0.0, -90 * deg, 0.0, 90 * deg, 45 * deg});
    return poses;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <robot_ip> [out.csv]\n";
        return 1;
    }
    const std::string robot_ip = argv[1];
    const std::string out_path = (argc >= 3) ? argv[2] : "franka_dynamics.csv";

    try {
        std::cout << "[franka_dump] Connecting to '" << robot_ip << "'...\n";
        franka::Robot robot(robot_ip);

        auto state = robot.readOnce();
        if (state.robot_mode == franka::RobotMode::kReflex ||
            static_cast<bool>(state.current_errors)) {
            std::cout << "[franka_dump] Recovering from error...\n";
            robot.automaticErrorRecovery();
            state = robot.readOnce();
        }

        // libfranka's model library. Its gravity uses the end-effector + load
        // configured in Desk (m_total, F_x_Ctotal) - with the Franka Hand set
        // as end effector this matches a URDF that includes the hand.
        franka::Model model = robot.loadModel();

        // Report the current real pose comparison too.
        const auto g_now = model.gravity(state);
        std::cout << "[franka_dump] Current pose q (rad): ";
        for (double v : state.q) std::cout << v << " ";
        std::cout << "\n[franka_dump] Franka g(q_current) (Nm): ";
        for (double v : g_now) std::cout << v << " ";
        std::cout << "\n[franka_dump] m_total=" << state.m_total << " kg, F_x_Ctotal=("
                  << state.F_x_Ctotal[0] << ", " << state.F_x_Ctotal[1] << ", "
                  << state.F_x_Ctotal[2] << ") m\n";

        std::ofstream csv(out_path);
        csv << "# j0..j6 (rad), g0..g6 (Nm) from libfranka model\n";
        // First row: the current real pose (so you can line it up to Desk).
        WriteRow(csv, state.q, g_now);

        for (const auto& q : BuildPoses()) {
            WriteRow(csv, q, model.gravity(q, state.m_total, state.F_x_Ctotal));
        }
        csv.close();
        std::cout << "[franka_dump] Wrote " << out_path << ".\n";
    } catch (const std::exception& e) {
        std::cerr << "[franka_dump] Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
