#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <lcm/lcm.h>

#include "communication/franka/BridgeConfig.hpp"
#include "communication/franka/FrankaClient.hpp"
#include "communication/franka/JointMapper.hpp"
#include "communication/franka/LcmCommandSubscriber.hpp"
#include "communication/franka/LcmStatePublisher.hpp"
#include "communication/franka/SafetyMonitor.hpp"

namespace communication::franka {

class FrankaBridge {
public:
    explicit FrankaBridge(BridgeConfig config);
    ~FrankaBridge();

    void Initialize();
    void Run(const std::atomic<bool>& stop_requested);

private:
    static double NowSeconds();

    BridgeConfig config_;
    FrankaClient client_;
    JointMapper mapper_;
    SafetyMonitor safety_monitor_;

    lcm_t* lcm_ = nullptr;
    std::unique_ptr<LcmCommandSubscriber> command_subscriber_;
    std::unique_ptr<LcmStatePublisher> state_publisher_;
};

}  // namespace communication::franka
