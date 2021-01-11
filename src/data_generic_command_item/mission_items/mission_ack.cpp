#include "mission_ack.h"

namespace MissionItem {

MissionACK::MissionACK(const unsigned int &ID, const  MAV_MISSION_RESULT &ack, const MissionKey &key, const MISSIONSTATE &state):
    m_SystemID(ID), result(ack), refKey(key), newState(state)
{

}

}
