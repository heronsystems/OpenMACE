#include "mission_key.h"

namespace MissionItem {

MissionKey::MissionKey():
    m_systemID(0),m_creatorID(0),m_missionID(0),m_missionType(MISSIONTYPE::AUTO),m_missionState(MISSIONSTATE::CURRENT)
{

}

MissionKey::MissionKey(const unsigned int &systemID, const unsigned int &creatorID, const unsigned int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &missionState):
    m_systemID(systemID),m_creatorID(creatorID),m_missionID(missionID),m_missionType(missionType),m_missionState(missionState)
{

}

MissionKey::MissionKey(const MissionKey &obj)
{
    this->m_systemID = obj.m_systemID;
    this->m_creatorID =obj.m_creatorID;
    this->m_missionID = obj.m_missionID;
    this->m_missionType = obj.m_missionType;
    this->m_missionState = obj.m_missionState;

}

MissionKey& MissionKey::operator =(const MissionKey &rhs)
{
    this->m_systemID = rhs.m_systemID;
    this->m_creatorID =rhs.m_creatorID;
    this->m_missionID = rhs.m_missionID;
    this->m_missionType = rhs.m_missionType;
    this->m_missionState = rhs.m_missionState;
    return *this;
}

bool MissionKey::operator <(const MissionKey &rhs) const
{
    if(*this == rhs)
        return false;

    if(this->m_systemID > rhs.m_systemID)
        return false;
    if(this->m_missionID > rhs.m_missionID)
        return false;
    if(this->m_creatorID > rhs.m_creatorID)
        return false;
    if(this->m_missionType > rhs.m_missionType)
        return false;
    if(this->m_missionState > rhs.m_missionState)
        return false;

    return true;

}

bool MissionKey::operator ==(const MissionKey &rhs) const
{
    if(this->m_systemID != rhs.m_systemID)
        return false;
    if(this->m_creatorID != rhs.m_creatorID)
        return false;
    if(this->m_missionID != rhs.m_missionID)
        return false;
    if(this->m_missionType != rhs.m_missionType)
        return false;
    if(this->m_missionState != rhs.m_missionState)
        return false;
    return true;
}

bool MissionKey::operator !=(const MissionKey &rhs) const
{
    return !((*this) == rhs);
}

std::ostream& operator<<(std::ostream& os, const MissionKey& t)
{
    std::stringstream stream;
    stream << std::fixed << "Mission Key: System ID " << std::to_string(t.m_systemID)
           << ", Creator ID"<< std::to_string(t.m_creatorID)
           << ", Mission ID " << std::to_string(t.m_missionID) << ".";
    os << stream.str();

    return os;
}

} //end of namespace Data
