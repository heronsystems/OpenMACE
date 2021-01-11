#ifndef MISSION_ACK_H
#define MISSION_ACK_H

#include <mavlink.h>
#include "mission_key.h"

namespace MissionItem {

class MissionACK
{

public:
    MissionACK(const unsigned int &systemID, const  MAV_MISSION_RESULT &ack, const MissionKey &key, const MISSIONSTATE &newState);

public:
    unsigned int getSystemID() const{
        return this->m_SystemID;
    }

     MAV_MISSION_RESULT getMissionResult() const{
        return this->result;
    }

    MissionKey getMissionKey() const{
        return this->refKey;
    }

    MISSIONSTATE getNewMissionState() const{
        return this->newState;
    }

    MissionKey getUpdatedMissionKey() const{
        MissionKey key = getMissionKey();
        key.m_missionState = getNewMissionState();
        return key;
    }

public:
    void operator = (const MissionACK &rhs)
    {
        this->m_SystemID = rhs.m_SystemID;
        this->result = rhs.result;
        this->refKey = rhs.refKey;
        this->newState = rhs.newState;
    }

    bool operator == (const MissionACK &rhs) {
        if(this->m_SystemID != rhs.m_SystemID){
            return false;
        }
        if(this->result != rhs.result){
            return false;
        }
        if(!(this->refKey != rhs.refKey)){
            return false;
        }
        if(this->newState != rhs.newState){
            return false;
        }
        return true;
    }

    bool operator != (const MissionACK &rhs) {
        return !(*this == rhs);
    }


private:

    unsigned int m_SystemID;

     MAV_MISSION_RESULT result;

    MissionKey refKey;

    MISSIONSTATE newState;
};

} //end of namespace MissionItem
#endif // MISSION_ACK_H
