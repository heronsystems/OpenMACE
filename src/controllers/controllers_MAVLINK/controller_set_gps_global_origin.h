#ifndef MAVLINK_CONTROLLER_SET_GPS_GLOBAL_ORIGIN_H
#define MAVLINK_CONTROLLER_SET_GPS_GLOBAL_ORIGIN_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "controllers/controllers_MAVLINK/common.h"

#include "data_generic_command_item/do_items/action_set_global_origin.h"

namespace MAVLINKUXVControllers {

using CONTROLLER_GLOBALORIGIN_TYPE = Controllers::GenericController<
    mavlink_message_t, MavlinkEntityKey,
    TransmitQueueWithKeys<MavlinkEntityKey, ObjectMAVLINKMsgIDTuple<MavlinkEntityKey>>,
    mace::pose::GeodeticPosition_3D,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::Action_SetGlobalOrigin>
>;

using GPSOriginSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    CONTROLLER_GLOBALORIGIN_TYPE,
    MavlinkEntityKey,
    command_item::Action_SetGlobalOrigin,
    mavlink_set_gps_global_origin_t,
    MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
>;

using GPSOriginFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    CONTROLLER_GLOBALORIGIN_TYPE,
    MavlinkEntityKey,
    mace::pose::GeodeticPosition_3D,
    mavlink_gps_global_origin_t,
    MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
>;

class Controller_SetGPSGlobalOrigin : public CONTROLLER_GLOBALORIGIN_TYPE,
        public GPSOriginSend,
        public GPSOriginFinish
{
private:

    std::unordered_map<MavlinkEntityKey, MavlinkEntityKey> m_CommandRequestedFrom;

protected:

    bool Construct_Send(const command_item::Action_SetGlobalOrigin &commandItem, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_set_gps_global_origin_t &gpsOriginItem, MavlinkEntityKey &queueObj) override
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetKeyFromSecondaryID(commandItem.getTargetSystem());

        gpsOriginItem = initializeGlobalOrigin();
        gpsOriginItem.target_system = static_cast<uint8_t>(commandItem.getTargetSystem());

        FillGPSOriginItem(commandItem,gpsOriginItem);

        return true;
    }


    bool Finish_Receive(const mavlink_gps_global_origin_t &msg, const MavlinkEntityKey &sender, mace::pose::GeodeticPosition_3D &ack, MavlinkEntityKey &queueObj) override
    {
        UNUSED(sender); UNUSED(queueObj);

        double power = std::pow(10,7);
        ack.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_AMSL);
        ack.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_MSL);
        ack.setAltitude(msg.altitude / 1000.0);
        ack.setLatitude(msg.latitude / power);
        ack.setLongitude(msg.longitude / power); //transforms it from mm to meters

        return true;
    }

protected:
    void FillGPSOriginItem(const command_item::Action_SetGlobalOrigin &commandItem, mavlink_set_gps_global_origin_t &mavlinkItem);

    mavlink_set_gps_global_origin_t initializeGlobalOrigin()
    {
        mavlink_set_gps_global_origin_t item;
        item.target_system = 0;
        item.latitude = 0.0;
        item.longitude = 0.0;
        item.altitude = 0.0;

        return item;
    }

public:
    Controller_SetGPSGlobalOrigin(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        CONTROLLER_GLOBALORIGIN_TYPE(cb, queue, linkChan, "SetGPSGlobalOrigin"),
        GPSOriginSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_gps_global_origin_t>(mavlink_msg_set_gps_global_origin_encode_chan)),
        GPSOriginFinish(this, mavlink_msg_gps_global_origin_decode)
    {

    }

    ~Controller_SetGPSGlobalOrigin() override = default;

};

} //end of namespace MAVLINKUXVControllers

#endif // MAVLINK_CONTROLLER_SET_GPS_GLOBAL_ORIGIN_H
