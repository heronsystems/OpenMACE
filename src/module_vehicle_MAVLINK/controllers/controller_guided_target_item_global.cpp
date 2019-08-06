#include "controller_guided_target_item_global.h"
#include "base/pose/pose_components.h"

namespace MAVLINKVehicleControllers {

    template <>
    void ControllerGuidedTargetItem_Global<TargetControllerStructGlobal>::FillTargetItem(const TargetControllerStructGlobal &targetStruct, mavlink_set_position_target_global_int_t &mavlinkItem)
    {
        double power = pow(10,7);
        uint16_t bitArray = 65535;

        //This handles the packing of the position components

        if(targetStruct.target.getPosition()->is2D())
        {
            const mace::pose::GeodeticPosition_2D* castPosition = targetStruct.target.getPosition()->positionAs<mace::pose::GeodeticPosition_2D>();
            mavlinkItem.lat_int = static_cast<int32_t>(castPosition->getLatitude() * power);
            if(castPosition->hasLatitudeBeenSet())
                bitArray = (bitArray & (~1)) | (static_cast<int>(0)<<0);
            mavlinkItem.lat_int = static_cast<int32_t>(castPosition->getLatitude() * power);
            if(castPosition->hasLongitudeBeenSet())
                bitArray = (bitArray & (~2)) | (static_cast<int>(0)<<1);
            mavlinkItem.alt = 0;

        }
        else if(targetStruct.target.getPosition()->is3D())
        {
            const mace::pose::GeodeticPosition_3D* castPosition = targetStruct.target.getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
            mavlinkItem.lat_int = static_cast<int32_t>(castPosition->getLatitude() * power);
            if(castPosition->hasLatitudeBeenSet())
                bitArray = (bitArray & (~1)) | (static_cast<int>(0)<<0);
            mavlinkItem.lat_int = static_cast<int32_t>(castPosition->getLatitude() * power);
            if(castPosition->hasLongitudeBeenSet())
                bitArray = (bitArray & (~2)) | (static_cast<int>(0)<<1);
            mavlinkItem.alt = static_cast<int32_t>(castPosition->getLatitude() * power);
            if(castPosition->hasAltitudeBeenSet())
                bitArray = (bitArray & (~4)) | (static_cast<int>(0)<<2);
        }

        mavlinkItem.type_mask = bitArray;
    }

}// end of namespace MAVLINKVehicleControllers
