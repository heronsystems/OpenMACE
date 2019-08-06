#ifndef MISSION_ACK_H
#define MISSION_ACK_H

#include "mace.h"
#include "mission_key.h"

namespace MissionItem {

class MissionACK
{
public:
    enum class MISSION_RESULT{
        MISSION_RESULT_ACCEPTED=0, /* mission accepted OK | */
        MISSION_RESULT_ERROR=1, /* generic error / not accepting mission commands at all right now | */
        MISSION_RESULT_UNSUPPORTED_FRAME=2, /* coordinate frame is not supported | */
        MISSION_RESULT_UNSUPPORTED=3, /* command is not supported | */
        MISSION_RESULT_NO_SPACE=4, /* mission item exceeds storage space | */
        MISSION_RESULT_INVALID=5, /* one of the parameters has an invalid value | */
        MISSION_RESULT_INVALID_SEQUENCE=13, /* received waypoint out of sequence | */
        MISSION_RESULT_DENIED=14, /* not accepting any mission commands from this communication partner | */
        MISSION_RESULT_DOES_NOT_EXIST=15, /* the requested mission with the associated key does not exist. | */
        MISSION_RESULT_RESULT_ENUM_END=16
    };

public:
    MissionACK(const unsigned int &systemID, const MISSION_RESULT &ack, const MissionKey &key, const MISSIONSTATE &newState);

public:
    unsigned int getSystemID() const{
        return this->m_SystemID;
    }

    MISSION_RESULT getMissionResult() const{
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

    MISSION_RESULT result;

    MissionKey refKey;

    MISSIONSTATE newState;
};

} //end of namespace MissionItem
#endif // MISSION_ACK_H
