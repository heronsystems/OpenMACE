#ifndef CONTROLLER_VISION_POSITION_ESTIMATE_H
#define CONTROLLER_VISION_POSITION_ESTIMATE_H

#include <mavlink.h>

#include "base/pose/pose.h"

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_broadcast.h"


#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"


namespace MAVLINKUXVControllers {

using namespace mace::pose;

using VisionPosititionBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<mace::pose::Pose>,
    mace::pose::Pose,
    mavlink_vision_position_estimate_t
>;


class Controller_VisionPositionEstimate : public BasicMavlinkController_ModuleKeyed<mace::pose::Pose>,
        public VisionPosititionBroadcast
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

    mavlink_set_position_target_global_int_t m_targetMSG;

private:
    bool doesMatchTransmitted(const mavlink_position_target_global_int_t &msg) const;

protected:

    void Construct_Broadcast(const mace::pose::Pose &poseEstimate, const MavlinkEntityKey &sender, mavlink_vision_position_estimate_t &visionEstimate) override
    {
        UNUSED(sender);

        visionEstimate = initializeMAVLINKVisionEstimate();
        FillVisionEstimate(poseEstimate,visionEstimate);
    }

protected:
    void FillVisionEstimate(const mace::pose::Pose &poseEstimate, mavlink_vision_position_estimate_t &mavlinkItem)
    {
        mavlinkItem.usec = poseEstimate.m_UpdateTime.ToMillisecondsSinceEpoch() * 1000;
        mavlinkItem.x = static_cast<float>(poseEstimate.m_Position.getXPosition());
        mavlinkItem.y = static_cast<float>(poseEstimate.m_Position.getYPosition());
        mavlinkItem.z = static_cast<float>(poseEstimate.m_Position.getZPosition());

        double roll,pitch,yaw;
        poseEstimate.m_Rotation.getDiscreteEuler(roll,pitch,yaw);
        mavlinkItem.roll = static_cast<float>(roll);
        mavlinkItem.pitch = static_cast<float>(pitch);
        mavlinkItem.yaw = static_cast<float>(yaw);
    }

    mavlink_vision_position_estimate_t initializeMAVLINKVisionEstimate()
    {
        mavlink_vision_position_estimate_t visionEstimate;
        visionEstimate.x = 0.0;
        visionEstimate.y = 0.0;
        visionEstimate.z = 0.0;

        visionEstimate.roll = 0.0;
        visionEstimate.pitch = 0.0;
        visionEstimate.yaw = 0.0;

        visionEstimate.usec = 0;
        visionEstimate.reset_counter = 0;

        return visionEstimate;
    }

public:
    Controller_VisionPositionEstimate(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<mace::pose::Pose>(cb, queue, linkChan),
        VisionPosititionBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_vision_position_estimate_t>(mavlink_msg_vision_position_estimate_encode_chan))
    {

    }

};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_VISION_POSITION_ESTIMATE_H
