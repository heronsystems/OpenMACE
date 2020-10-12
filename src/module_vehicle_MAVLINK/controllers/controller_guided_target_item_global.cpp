#include "controller_guided_target_item_global.h"
#include "base/pose/pose_components.h"

/*
Coordinate_frame valid options are:

MAV_FRAME_GLOBAL_INT : alt is meters above sea level
MAV_FRAME_GLOBAL_RELATIVE_ALT: alt is meters above home
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: alt is meters above home
MAV_FRAME_GLOBAL_RELATIVE_TERRAIN_ALT: alt is meters above terrain
MAV_FRAME_GLOBAL_RELATIVE_TERRAIN_ALT_INT: alt is meters above terrain
*/

namespace MAVLINKUXVControllers {

void ControllerGuidedTargetItem_Global::FillTargetItem(const command_target::DynamicTarget_Kinematic &command, mavlink_set_position_target_global_int_t &mavlinkItem)
{
    double power = pow(10,7);
    uint16_t bitArray = 3583; // first let us assume that they are all

    command_target::DynamicTarget_Kinematic currentTarget = command;

    mavlinkItem.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

    //This handles the packing of the position components
    if((currentTarget.getPosition() != nullptr) && (currentTarget.getPosition()->isAnyPositionValid()))
    {
        if(currentTarget.getPosition()->is2D())
        {
            mace::pose::GeodeticPosition_2D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::GeodeticPosition_2D>();
            if(castPosition->hasLatitudeBeenSet()) {
                mavlinkItem.lat_int = static_cast<int32_t>(castPosition->getLatitude() * power);
                bitArray = (bitArray & (~1));
            }
            if(castPosition->hasLongitudeBeenSet()) {
                mavlinkItem.lon_int = static_cast<int32_t>(castPosition->getLongitude() * power);
                bitArray = (bitArray & (~2));
            }
        }
        else if(currentTarget.getPosition()->is3D())
        {
            mace::pose::GeodeticPosition_3D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
            if(castPosition->hasLatitudeBeenSet()) {
                mavlinkItem.lat_int = static_cast<int32_t>(castPosition->getLatitude() * power);
                bitArray = (bitArray & (~1));
            }
            if(castPosition->hasLongitudeBeenSet()) {
                mavlinkItem.lon_int = static_cast<int32_t>(castPosition->getLongitude() * power);
                bitArray = (bitArray & (~2));
            }
            if(castPosition->hasAltitudeBeenSet()) {
                mavlinkItem.alt = static_cast<float>(castPosition->getAltitude());
                bitArray = (bitArray & (~4));
            }
        }
    }

    //This handles the packing of the velocity components
    if((currentTarget.getVelocity() != nullptr) && (currentTarget.getVelocity()->isAnyVelocityValid()))
    {
        if(currentTarget.getVelocity()->is2D())
        {
            mace::pose::Velocity_Cartesian2D* castVelocity = currentTarget.getVelocity()->velocityAs<mace::pose::Velocity_Cartesian2D>();
            if(castVelocity->hasXBeenSet()) {
                mavlinkItem.vx = static_cast<float>(castVelocity->getXVelocity());
                bitArray = (bitArray & (~8));
            }
            if(castVelocity->hasYBeenSet()) {
                mavlinkItem.vy = static_cast<float>(castVelocity->getYVelocity());
                bitArray = (bitArray & (~16));
            }
        }
        else if(currentTarget.getVelocity()->is3D())
        {
            mace::pose::Velocity_Cartesian3D* castVelocity = currentTarget.getVelocity()->velocityAs<mace::pose::Velocity_Cartesian3D>();
            if(castVelocity->hasXBeenSet()) {
                mavlinkItem.vx = static_cast<float>(castVelocity->getXVelocity());
                bitArray = (bitArray & (~8));
            }
            if(castVelocity->hasYBeenSet()) {
                mavlinkItem.vy = static_cast<float>(castVelocity->getYVelocity());
                bitArray = (bitArray & (~16));
            }
            if(castVelocity->hasZBeenSet()) {
                mavlinkItem.vz = static_cast<float>(castVelocity->getZVelocity());
                bitArray = (bitArray & (~32));
            }
        }
    }

    if((currentTarget.getYaw() != nullptr) && (currentTarget.getYaw()->isYawDimensionSet()))
    {
        mace::pose::Rotation_2D* castRotation = currentTarget.getYaw();
        mavlinkItem.yaw = static_cast<float>(castRotation->getPhi()); bitArray = (bitArray & (~1024));
    }

    if((currentTarget.getYawRate() != nullptr) && (currentTarget.getYawRate()->isYawDimensionSet()))
    {
        mace::pose::Rotation_2D* castRotation = currentTarget.getYawRate();
        mavlinkItem.yaw_rate = static_cast<float>(castRotation->getPhi()); bitArray = (bitArray & (~2048));
    }
    mavlinkItem.type_mask = bitArray;
}

bool ControllerGuidedTargetItem_Global::doesMatchTransmitted(const mavlink_position_target_global_int_t &msg) const
{
    if(fabsf(m_targetMSG.vx - msg.vx) > std::numeric_limits<float>::epsilon())
        return false;
    if(fabsf(m_targetMSG.vy - msg.vy) > std::numeric_limits<float>::epsilon())
        return false;
    if(fabsf(m_targetMSG.vz - msg.vz) > std::numeric_limits<float>::epsilon())
        return false;

//    if(fabsf(m_targetMSG.alt - msg.alt) > std::numeric_limits<float>::epsilon())
//        return false;

    if(fabsf(m_targetMSG.yaw - msg.yaw) > std::numeric_limits<float>::epsilon())
        return false;
    if(fabsf(m_targetMSG.yaw_rate - msg.yaw_rate) > std::numeric_limits<float>::epsilon())
        return false;

    if(m_targetMSG.lat_int != msg.lat_int)
        return false;
    if(m_targetMSG.lon_int != msg.lon_int)
        return false;

//    if(m_targetMSG.coordinate_frame != msg.coordinate_frame)
//        return false;
    if(m_targetMSG.type_mask != msg.type_mask)
        return false;

    return true;
}
}// end of namespace MAVLINKVehicleControllers
