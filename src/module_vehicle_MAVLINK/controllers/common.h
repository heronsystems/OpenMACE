#ifndef MODULE_VEHICLE_MAVLINK_CONTROLLERS_COMMON_H
#define MODULE_VEHICLE_MAVLINK_CONTROLLERS_COMMON_H

#include "common/object_int_tuple.h"

#include "controllers/generic_controller.h"

#include "mavlink.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

namespace MAVLINKUXVControllers {

//!
//! \brief ObjectMAVLINKMsgIDTuple And class that wraps up some object with an integer identifying the mace message ID
//!
template <typename T>
using ObjectMAVLINKMsgIDTuple = ObjectIntTuple<T>;


template <typename T>
using BasicMavlinkController_ModuleKeyed = Controllers::GenericController<
    mavlink_message_t, MavlinkEntityKey,
    TransmitQueueWithKeys<MavlinkEntityKey, ObjectMAVLINKMsgIDTuple<MavlinkEntityKey>>,
    uint8_t,
    Controllers::DataItem<MavlinkEntityKey, T>
>;


template<typename T>
std::function<uint16_t(MavlinkEntityKey, uint8_t, mavlink_message_t*, const T*)> MavlinkEntityKeyToSysIDCompIDConverter(const std::function<uint16_t(uint8_t, uint8_t, uint8_t, mavlink_message_t*, const T*)> &lambda){
    return [lambda](MavlinkEntityKey sender, uint8_t chan, mavlink_message_t* msg, const T* data){
        uint8_t sysID = sender;
        uint8_t compID = 0;
        return lambda(sysID, compID, chan, msg, data);
    };
}

}

#endif // MODULE_VEHICLE_MAVLINK_CONTROLLERS_COMMON_H
