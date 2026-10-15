#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <string>

#include "communication/franka/BridgeConfig.hpp"
#include "communication/franka/FrankaBridge.hpp"

namespace {

std::atomic<bool> g_stop_requested{false};

void HandleSignal(int /*signum*/)
{
    g_stop_requested.store(true);
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string default_config =
        "configs/franka_fr3/hardware/bridge.yaml";
    const std::string config_path = (argc > 1) ? argv[1] : default_config;

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    try {
        auto config =
            communication::franka::BridgeConfig::LoadFromFile(config_path);
        communication::franka::FrankaBridge bridge(std::move(config));
        bridge.Initialize();
        bridge.Run(g_stop_requested);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "[franka_bridge] Fatal error: " << ex.what() << std::endl;
        return 1;
    }
}
