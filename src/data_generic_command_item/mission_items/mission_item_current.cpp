#include "mission_item_current.h"

namespace MissionItem {

MissionItemCurrent::MissionItemCurrent()
{

}

MissionItemCurrent::MissionItemCurrent(const MissionKey &missionKey, const unsigned int &index):
    key(missionKey), indexCurrent(index)
{

}

MissionItemCurrent::MissionItemCurrent(const mavlink_mission_current_t &obj)
{
//    key.m_creatorID = obj.mission_creator;
//    key.m_missionID = obj.mission_id;
//    key.m_missionState = static_cast<MISSIONSTATE>(obj.mission_state);
//    key.m_missionType = static_cast<MISSIONTYPE>(obj.mission_type);
//    key.m_systemID = obj.target_system;
    indexCurrent = obj.seq;
}

mavlink_mission_current_t MissionItemCurrent::getMACECommsObject() const
{
    mavlink_mission_current_t rtn;
//    rtn.mission_creator = static_cast<uint8_t>(key.m_creatorID);
//    rtn.mission_id = static_cast<uint8_t>(key.m_missionID);
//    rtn.mission_state = static_cast<uint8_t>(key.m_missionState);
//    rtn.mission_type = static_cast<uint8_t>(key.m_missionType);
//    rtn.mission_system = static_cast<uint8_t>(key.m_systemID);
    rtn.seq = static_cast<uint16_t>(indexCurrent);
    return rtn;
}

mavlink_message_t MissionItemCurrent::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_mission_current_t current = getMACECommsObject();
    mavlink_message_t msg;
    mavlink_msg_mission_current_encode_chan(systemID,compID,chan,&msg,&current);
    return msg;
}

} //end of namespace MissionItem
