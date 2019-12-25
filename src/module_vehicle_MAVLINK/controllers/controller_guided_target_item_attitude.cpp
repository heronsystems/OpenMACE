#include "controller_guided_target_item_attitude.h"

namespace MAVLINKUXVControllers {

void ControllerGuidedTargetItem_Attitude::FillTargetItem(const command_target::DynamicTarget_Orientation &command, mavlink_set_attitude_target_t &mavlinkItem)
{
    uint8_t bitArray = 255; // first let us assume that they are all

    const mace::pose::AbstractRotation* currentRotation = command.getTargetOrientation();

    if(currentRotation != nullptr)
    {
        bitArray = (bitArray & (~128));
        Eigen::Quaterniond quat = currentRotation->getQuaternion();
        mavlinkItem.q[0] = static_cast<float>(quat.w());
        mavlinkItem.q[1] = static_cast<float>(quat.x());
        mavlinkItem.q[2] = static_cast<float>(quat.y());
        mavlinkItem.q[3] = static_cast<float>(quat.z());
    }

    if(fabs(command.getTargetThrust() - -1000) > std::numeric_limits<double>::epsilon())
    {
        bitArray = (bitArray & (~64));
        mavlinkItem.thrust = static_cast<float>(command.getTargetThrust());
    }

    if(fabs(command.getTargetYawRate() - 0.0) > std::numeric_limits<double>::epsilon())
    {
        bitArray = (bitArray & (~4));
        mavlinkItem.body_yaw_rate = static_cast<float>(command.getTargetYawRate());
    }

    mavlinkItem.type_mask = bitArray;
}

}// end of namespace MAVLINKVehicleControllers

