#include "mission_item_current.h"

namespace MissionItem {

MissionItemCurrent::MissionItemCurrent()
{

}

MissionItemCurrent::MissionItemCurrent(const MissionKey &missionKey, const int &index):
    key(missionKey), indexCurrent(index)
{

}

MissionItemCurrent::MissionItemCurrent(const mace_mission_item_current_t &obj)
{
    key.m_creatorID = obj.mission_creator;
    key.m_missionID = obj.mission_id;
    key.m_missionState = static_cast<MISSIONSTATE>(obj.mission_state);
    key.m_missionType = static_cast<MISSIONTYPE>(obj.mission_type);
    key.m_systemID = obj.mission_system;
    indexCurrent = obj.seq;
}

mace_mission_item_current_t MissionItemCurrent::getMACECommsObject() const
{
    mace_mission_item_current_t rtn;
    rtn.mission_creator = key.m_creatorID;
    rtn.mission_id = key.m_missionID;
    rtn.mission_state = (uint8_t)key.m_missionState;
    rtn.mission_type = (uint8_t)key.m_missionType;
    rtn.mission_system = key.m_systemID;
    rtn.seq = indexCurrent;
    return rtn;
}

mace_message_t MissionItemCurrent::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_mission_item_current_t current = getMACECommsObject();
    mace_message_t msg;
    mace_msg_mission_item_current_encode_chan(systemID,compID,chan,&msg,&current);
    return msg;
}

} //end of namespace MissionItem
