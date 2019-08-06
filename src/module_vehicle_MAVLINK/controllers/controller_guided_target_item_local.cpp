#include "controller_guided_target_item_local.h"
#include "base/pose/pose_components.h"

namespace MAVLINKVehicleControllers {

template <>
void ControllerGuidedTargetItem_Local<TargetControllerStructLocal>::FillTargetItem(const TargetControllerStructLocal &targetStruct, mavlink_set_position_target_local_ned_t &mavlinkItem)
{
    double power = pow(10,7);
    uint16_t bitArray = 65535;

    //This handles the packing of the position components

    if(targetStruct.target.getPosition()->is2D())
    {
        const mace::pose::CartesianPosition_2D* castPosition = targetStruct.target.getPosition()->positionAs<mace::pose::CartesianPosition_2D>();
        mavlinkItem.x = static_cast<int32_t>(castPosition->getXPosition());
        if(castPosition->hasXBeenSet())
            bitArray = (bitArray & (~1)) | (static_cast<int>(0)<<0);
        mavlinkItem.y = static_cast<int32_t>(castPosition->getYPosition());
        if(castPosition->hasYBeenSet())
            bitArray = (bitArray & (~2)) | (static_cast<int>(0)<<1);
        mavlinkItem.z = 0;

    }
    else if(targetStruct.target.getPosition()->is3D())
    {
        const mace::pose::CartesianPosition_3D* castPosition = targetStruct.target.getPosition()->positionAs<mace::pose::CartesianPosition_3D>();
        mavlinkItem.x = static_cast<int32_t>(castPosition->getXPosition() * power);
        if(castPosition->hasXBeenSet())
            bitArray = (bitArray & (~1)) | (static_cast<int>(0)<<0);
        mavlinkItem.y = static_cast<int32_t>(castPosition->getYPosition() * power);
        if(castPosition->hasYBeenSet())
            bitArray = (bitArray & (~2)) | (static_cast<int>(0)<<1);
        mavlinkItem.z = static_cast<int32_t>(castPosition->getZPosition() * power);
        if(castPosition->hasZBeenSet())
            bitArray = (bitArray & (~4)) | (static_cast<int>(0)<<2);
    }

    mavlinkItem.type_mask = bitArray;
}

}// end of namespace MAVLINKVehicleControllers
