#include "controller_guided_target_item_global.h"
#include "base/pose/pose_components.h"

namespace MAVLINKVehicleControllers {

    template <>
    void ControllerGuidedTargetItem_Global<TargetControllerStructGlobal>::FillTargetItem(const TargetControllerStructGlobal &targetStruct, mavlink_set_position_target_global_int_t &mavlinkItem)
    {
        double power = pow(10,7);
        uint16_t bitArray = 65535; // first let us assume that they are all

        command_target::DynamicTarget currentTarget = targetStruct.target;

        //This handles the packing of the position components
        if(currentTarget.getPosition()->isAnyPositionValid())
        {
            if(currentTarget.getPosition()->is2D())
            {
                mace::pose::GeodeticPosition_2D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::GeodeticPosition_2D>();
                if(castPosition->hasLatitudeBeenSet())
                    mavlinkItem.lat_int = castPosition->getLatitude() * power; bitArray = (bitArray & (~1));
                if(castPosition->hasLongitudeBeenSet())
                    mavlinkItem.lon_int = castPosition->getLongitude() * power; bitArray = (bitArray & (~2));
            }
            else if(currentTarget.getPosition()->is3D())
            {
                mace::pose::GeodeticPosition_3D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
                if(castPosition->hasLatitudeBeenSet())
                    mavlinkItem.lat_int = castPosition->getLatitude() * power; bitArray = (bitArray & (~1));
                if(castPosition->hasLongitudeBeenSet())
                    mavlinkItem.lon_int = castPosition->getLongitude() * power; bitArray = (bitArray & (~2));
                if(castPosition->hasAltitudeBeenSet())
                    mavlinkItem.alt = castPosition->getAltitude();  bitArray = (bitArray & (~4));
            }
        }
        if(targetStruct.target.getVelocity()->isAnyVelocityValid())
        {
            if(currentTarget.getVelocity()->is2D())
            {
                mace::pose::Geodetic_Velocity2D* castVelocity = currentTarget.getPosition()->positionAs<mace::pose::Geodetic_Velocity2D>();

                if(castPosition->h())
                    mavlinkItem.lat_int = castPosition->getLatitude() * power; bitArray = (bitArray & (~1));
                if(castPosition->hasLongitudeBeenSet())
                    mavlinkItem.lon_int = castPosition->getLongitude() * power; bitArray = (bitArray & (~2));
            }
            else if(currentTarget.getVelocity()->is3D())
            {
                mace::pose::Geodetic_Velocity3D* castVelocity = currentTarget.getPosition()->positionAs<mace::pose::Geodetic_Velocity3D>();
                if(castPosition->hasLatitudeBeenSet())
                    mavlinkItem.lat_int = castPosition->getLatitude() * power; bitArray = (bitArray & (~1));
                if(castPosition->hasLongitudeBeenSet())
                    mavlinkItem.lon_int = castPosition->getLongitude() * power; bitArray = (bitArray & (~2));
                if(castPosition->hasAltitudeBeenSet())
                    mavlinkItem.alt = castPosition->getAltitude();  bitArray = (bitArray & (~4));
            }

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
