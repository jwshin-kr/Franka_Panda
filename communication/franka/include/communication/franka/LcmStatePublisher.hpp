#pragma once

#include <string>

#include <lcm/lcm.h>

#include "communication/franka/BridgeTypes.hpp"

namespace communication::franka {

class LcmStatePublisher {
public:
    LcmStatePublisher(lcm_t* lcm, std::string joint_meas_channel);

    bool Publish(const ArmState& state) const;

private:
    lcm_t* lcm_ = nullptr;
    std::string joint_meas_channel_;
};

}  // namespace communication::franka
