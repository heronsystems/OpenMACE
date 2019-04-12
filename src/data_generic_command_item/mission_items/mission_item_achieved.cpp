#include "mission_item_achieved.h"

namespace MissionItem {

MissionItemAchieved::MissionItemAchieved()
{

}

MissionItemAchieved::MissionItemAchieved(const MissionKey &missionKey, const int &index):
    key(missionKey), indexAchieved(index)
{

}


MissionItemAchieved::MissionItemAchieved(const mace_mission_item_reached_t &obj)
{
    key.m_creatorID = obj.mission_creator;
    key.m_missionID = obj.mission_id;
    key.m_missionState = static_cast<MISSIONSTATE>(obj.mission_state);
    key.m_missionType = static_cast<MISSIONTYPE>(obj.mission_type);
    key.m_systemID = obj.mission_system;
    indexAchieved = obj.seq;
}

mace_mission_item_reached_t MissionItemAchieved::getMACECommsObject() const
{
    mace_mission_item_reached_t rtn;
    rtn.mission_creator = key.m_creatorID;
    rtn.mission_id = key.m_missionID;
    rtn.mission_state = (uint8_t)key.m_missionState;
    rtn.mission_type = (uint8_t)key.m_missionType;
    rtn.mission_system = key.m_systemID;
    rtn.seq = indexAchieved;
    return rtn;
}

mace_message_t MissionItemAchieved::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_mission_item_reached_t reached = getMACECommsObject();
    mace_message_t msg;
    mace_msg_mission_item_reached_encode_chan(systemID, compID, chan, &msg, &reached);
    return msg;
}

} //end of namespace MissionItem
