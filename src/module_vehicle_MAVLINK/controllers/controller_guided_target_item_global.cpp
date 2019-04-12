#include "controller_guided_target_item_global.h"

namespace MAVLINKVehicleControllers {

    template <>
    void ControllerGuidedTargetItem_Global<TargetControllerStructGlobal>::FillTargetItem(const TargetControllerStructGlobal &targetStruct, mavlink_set_position_target_global_int_t &mavlinkItem)
    {
        TargetItem::GeodeticDynamicTarget targetItem = targetStruct.target;

        mace::pose::GeodeticPosition_3D targetPosition = targetItem.getPosition();
        mace::pose::CartesianVelocity_3D targetVelocity = targetItem.getVelocity();

        mavlinkItem.lat_int = targetPosition.getLatitude() * pow(10,7);
        mavlinkItem.lon_int = targetPosition.getLongitude() * pow(10,7);
        mavlinkItem.alt = targetPosition.getAltitude();
        mavlinkItem.vx = targetVelocity.getXVelocity();
        mavlinkItem.vy = targetVelocity.getYVelocity();
        mavlinkItem.vz = targetVelocity.getZVelocity();
        mavlinkItem.yaw = targetItem.getYaw();
        mavlinkItem.yaw_rate = targetItem.getYawRate();
        //now we need to update the bitmask to the appropriate values
        //If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration.
        /*
         * Mapping: bit 1: x, bit 2: y, bit 3: z
         * bit 4: vx, bit 5: vy, bit 6: vz,
         * bit 7: ax, bit 8: ay, bit 9: az,
         * bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
         */

        uint16_t bitArray = 65535;
        uint16_t mask = 1<<0; // set the mask for a x position
        if(targetPosition.hasLatitudeBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<0);
        mask = 1<<1;
        if(targetPosition.hasLongitudeBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<1);
        mask = 1<<2;
        if(targetPosition.hasAltitudeBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<2);

        mask = 1<<3;
        if(targetVelocity.hasXVelocityBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<3);
        mask = 1<<4;
        if(targetVelocity.hasYVelocityBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<4);
        mask = 1<<5;
        if(targetVelocity.hasZVelocityBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<5);

        mask = 1<<10;
        if(std::abs(targetItem.getYaw()) > 0.001) //KEN FIX THIS
            bitArray = (bitArray & (~mask)) | ((int)0<<10);
        mask = 1<<11;
        if(std::abs(targetItem.getYawRate()) > 0.001) //KEN FIX THIS
            bitArray = (bitArray & (~mask)) | ((int)0<<11);

        mavlinkItem.type_mask = bitArray;
    }

}// end of namespace MAVLINKVehicleControllers
