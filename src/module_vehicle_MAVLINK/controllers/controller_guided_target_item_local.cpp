#include "controller_guided_target_item_local.h"
#include "base/pose/pose_components.h"

namespace MAVLINKVehicleControllers {

template <>
void ControllerGuidedTargetItem_Local<TargetControllerStructLocal>::FillTargetItem(const TargetControllerStructLocal &targetStruct, mavlink_set_position_target_local_ned_t &mavlinkItem)
{
    double power = pow(10,7);
    uint16_t bitArray = 65535;
    command_target::DynamicTarget currentTarget = targetStruct.target;

    //This handles the packing of the position components
//    if(currentTarget.getPosition()->isAnyPositionValid())
//    {
//        if(currentTarget.getPosition()->is2D())
//        {
//            mace::pose::GeodeticPosition_2D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::GeodeticPosition_2D>();
//            if(castPosition->hasLatitudeBeenSet())
//                mavlinkItem.lat_int = castPosition->getLatitude() * power; bitArray = (bitArray & (~1));
//            if(castPosition->hasLongitudeBeenSet())
//                mavlinkItem.lon_int = castPosition->getLongitude() * power; bitArray = (bitArray & (~2));
//        }
//        else if(currentTarget.getPosition()->is3D())
//        {
//            mace::pose::GeodeticPosition_3D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
//            if(castPosition->hasLatitudeBeenSet())
//                mavlinkItem.lat_int = castPosition->getLatitude() * power; bitArray = (bitArray & (~1));
//            if(castPosition->hasLongitudeBeenSet())
//                mavlinkItem.lon_int = castPosition->getLongitude() * power; bitArray = (bitArray & (~2));
//            if(castPosition->hasAltitudeBeenSet())
//                mavlinkItem.alt = castPosition->getAltitude();  bitArray = (bitArray & (~4));
//        }
//    }

    mavlinkItem.type_mask = bitArray;
}

}// end of namespace MAVLINKVehicleControllers
