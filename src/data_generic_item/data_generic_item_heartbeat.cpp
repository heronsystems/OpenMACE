#include "data_generic_item_heartbeat.h"

namespace DataGenericItem {

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat() :
    autopilot(Data::AutopilotType::AUTOPILOT_TYPE_GENERIC),protocol(Data::CommsProtocol::COMMS_MACE),type(Data::SystemType::SYSTEM_TYPE_GENERIC),
    missionState(Data::MissionExecutionState::MESTATE_UNEXECUTED),maceCompanion(true)
{

}

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj)
{
    this->protocol = copyObj.getProtocol();
    this->type = copyObj.getType();
    this->autopilot = copyObj.getAutopilot();
    this->missionState = copyObj.getMissionState();
    this->maceCompanion = copyObj.getCompanion();
    this->mavlinkID = copyObj.getMavlinkID();
}


mace_heartbeat_t DataGenericItem_Heartbeat::getMACECommsObject() const
{
    mace_heartbeat_t rtnObj;

    rtnObj.autopilot = (uint8_t)this->autopilot;
    rtnObj.mace_companion = this->maceCompanion;
    rtnObj.protocol = (uint8_t)this->protocol;
    rtnObj.type = (uint8_t)this->type;
    rtnObj.mavlinkID = this->mavlinkID;

    return rtnObj;
}

mace_message_t DataGenericItem_Heartbeat::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_heartbeat_t heartbeat = getMACECommsObject();
    mace_msg_heartbeat_encode_chan(systemID,compID,chan,&msg,&heartbeat);
    return msg;
}

} //end of namespace DataGenericItem
