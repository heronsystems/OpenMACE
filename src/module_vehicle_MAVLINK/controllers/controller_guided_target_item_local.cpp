#include "controller_guided_target_item_local.h"
#include "base/pose/pose_components.h"

/*
MAV_FRAME_LOCAL_NED
Positions are relative to the vehicle’s EKF Origin in NED frame

I.e x=1,y=2,z=3 is 1m North, 2m East and 3m Down from the origin

The EKF origin is the vehicle’s location when it first achieved a good position estimate

Velocity are in NED frame

MAV_FRAME_LOCAL_OFFSET_NED
Positions are relative to the vehicle’s current position

I.e. x=1,y=2,z=3 is 1m North, 2m East and 3m below the current position.

Velocity are in NED frame.

MAV_FRAME_BODY_OFFSET_NED
Positions are relative to the vehicle’s current position and heading

I.e x=1,y=2,z=3 is 1m forward, 2m right and 3m Down from the current position

Velocities are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).

MAV_FRAME_BODY_NED
Positions are relative to the EKF Origin in NED frame

I.e x=1,y=2,z=3 is 1m North, 2m East and 3m Down from the origin

Velocities are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).
*/

namespace MAVLINKUXVControllers {

void ControllerGuidedTargetItem_Local::FillTargetItem(const command_target::DynamicTarget_Kinematic &command, mavlink_set_position_target_local_ned_t &mavlinkItem)
{
    uint16_t bitArray = 65535;
    command_target::DynamicTarget_Kinematic currentTarget = command;

    mavlinkItem.coordinate_frame = MAV_FRAME_LOCAL_NED;

    //This handles the packing of the position components
    if((currentTarget.getPosition() != nullptr) && (currentTarget.getPosition()->isAnyPositionValid()))
    {
        if(currentTarget.getPosition()->is2D())
        {
            mace::pose::CartesianPosition_2D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::CartesianPosition_2D>();
            mavlinkItem.coordinate_frame = getMAVLINKCoordinateFrame(castPosition->getExplicitCoordinateFrame());
            if(castPosition->getCartesianCoordinateFrame() == mace::CartesianFrameTypes::CF_LOCAL_NED) //if not in the body frame we can move it
                castPosition->applyTransformation(m_vehicleHomeTOswarm);
            if(castPosition->hasXBeenSet()) {
                mavlinkItem.x = static_cast<float>(castPosition->getXPosition());
                bitArray = (bitArray & (~1));
            }
            if(castPosition->hasYBeenSet()) {
                mavlinkItem.y = static_cast<float>(castPosition->getYPosition());
                bitArray = (bitArray & (~2));
            }
        }
        else if(currentTarget.getPosition()->is3D())
        {
            mace::pose::CartesianPosition_3D* castPosition = currentTarget.getPosition()->positionAs<mace::pose::CartesianPosition_3D>();
            mavlinkItem.coordinate_frame = getMAVLINKCoordinateFrame(castPosition->getExplicitCoordinateFrame());
            if(castPosition->getCartesianCoordinateFrame() == mace::CartesianFrameTypes::CF_LOCAL_NED) //if not in the body frame we can move it
                castPosition->applyTransformation(m_vehicleHomeTOswarm);
            if(castPosition->hasXBeenSet()) {
                mavlinkItem.x = static_cast<float>(castPosition->getXPosition());
                bitArray = (bitArray & (~1));
            }
            if(castPosition->hasYBeenSet()) {
                mavlinkItem.y = static_cast<float>(castPosition->getYPosition());
                bitArray = (bitArray & (~2));
            }
            if(castPosition->hasZBeenSet()) {
                mavlinkItem.z = static_cast<float>(castPosition->getAltitude());
                bitArray = (bitArray & (~4));
            }
            std::cout<<"The target position is: "<<*castPosition<<std::endl;
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
            mavlinkItem.coordinate_frame = getMAVLINKCoordinateFrame(castVelocity->getExplicitCoordinateFrame());
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
    mavlinkItem.type_mask = bitArray;
}

bool ControllerGuidedTargetItem_Local::doesMatchTransmitted(const mavlink_position_target_local_ned_t &msg) const
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

    if(fabs(static_cast<double>(m_targetMSG.x) - static_cast<double>(msg.x)) > 0.1)
        return false;
    if(fabs(static_cast<double>(m_targetMSG.y) - static_cast<double>(msg.y)) > 0.1)
        return false;

//    if(m_targetMSG.coordinate_frame != msg.coordinate_frame)
//        return false;
    if(m_targetMSG.type_mask != msg.type_mask)
        return false;

    return true;
}
}// end of namespace MAVLINKVehicleControllers
