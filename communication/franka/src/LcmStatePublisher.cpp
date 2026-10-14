#include "communication/franka/LcmStatePublisher.hpp"

#include <utility>

#include "communication/franka/LcmCodec.hpp"

namespace communication::franka {

LcmStatePublisher::LcmStatePublisher(
    lcm_t* lcm, std::string joint_meas_channel)
    : lcm_(lcm), joint_meas_channel_(std::move(joint_meas_channel))
{
}

bool LcmStatePublisher::Publish(const ArmState& state) const
{
    if (lcm_ == nullptr) {
        return false;
    }

    const auto encoded = JointMeasLcmCodec::Encode(state);
    return lcm_publish(
               lcm_,
               joint_meas_channel_.c_str(),
               encoded.data(),
               static_cast<unsigned int>(encoded.size()))
        == 0;
}

}  // namespace communication::franka
