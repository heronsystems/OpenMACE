#include "mission_item_achieved.h"

namespace MissionItem {

MissionItemAchieved::MissionItemAchieved()
{

}

MissionItemAchieved::MissionItemAchieved(const MissionKey &missionKey, const unsigned int &index):
    key(missionKey), indexAchieved(index)
{

}


MissionItemAchieved::MissionItemAchieved(const mavlink_mission_item_reached_t &obj)
{
//    key.m_creatorID = obj.mission_creator;
//    key.m_missionID = obj.mission_id;
//    key.m_missionState = static_cast<MISSIONSTATE>(obj.mission_state);
//    key.m_missionType = static_cast<MISSIONTYPE>(obj.mission_type);
//    key.m_systemID = obj.mission_system;
    indexAchieved = obj.seq;
}

mavlink_mission_item_reached_t MissionItemAchieved::getMACECommsObject() const
{
    mavlink_mission_item_reached_t rtn;
//    rtn.mission_creator = static_cast<uint8_t>(key.m_creatorID);
//    rtn.mission_id = static_cast<uint8_t>(key.m_missionID);
//    rtn.mission_state = static_cast<uint8_t>(key.m_missionState);
//    rtn.mission_type = static_cast<uint8_t>(key.m_missionType);
//    rtn.mission_system = static_cast<uint8_t>(key.m_systemID);
    rtn.seq = static_cast<uint16_t>(indexAchieved);
    return rtn;
}

mavlink_message_t MissionItemAchieved::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_mission_item_reached_t reached = getMACECommsObject();
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_encode_chan(systemID, compID, chan, &msg, &reached);
    return msg;
}

} //end of namespace MissionItem
