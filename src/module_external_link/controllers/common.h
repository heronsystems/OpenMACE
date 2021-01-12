#ifndef GENERIC_EXTERNAL_LINK_CONTROLLER_H
#define GENERIC_EXTERNAL_LINK_CONTROLLER_H

#include "controllers/generic_controller.h"

#include <functional>

namespace ExternalLink{

//!
//! \brief ObjectMaceMsgIDTuple And class that wraps up some object with an integer identifying the mace message ID
//!
template <typename T>
using ObjectMaceMsgIDTuple = ObjectIntTuple<T>;


template <typename T>
using BasicExternalLinkController_ModuleKeyed = Controllers::GenericController<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectMaceMsgIDTuple<MaceCore::ModuleCharacteristic>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, T>>;

}


template<typename T>
std::function<uint16_t(MaceCore::ModuleCharacteristic, uint8_t, mavlink_message_t*, const T*)> ModuleToSysIDCompIDConverter(const std::function<uint16_t(uint8_t, uint8_t, uint8_t, mavlink_message_t*, const T*)> &lambda){
    return [lambda](MaceCore::ModuleCharacteristic sender, uint8_t chan, mavlink_message_t* msg, const T* data){
        uint8_t sysID = sender.MaceInstance;
        uint8_t compID = sender.ModuleID;
        return lambda(sysID, compID, chan, msg, data);
    };
}

#endif // GENERIC_EXTERNAL_LINK_CONTROLLER_H
