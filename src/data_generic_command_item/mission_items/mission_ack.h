#ifndef MISSION_ACK_H
#define MISSION_ACK_H

#include "mace.h"
#include "mission_key.h"

namespace MissionItem {

class MissionACK
{
public:
    enum class MISSION_RESULT{
        MISSION_RESULT_ACCEPTED=MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED, /* mission accepted OK | */
        MISSION_RESULT_ERROR=MAV_MISSION_RESULT::MAV_MISSION_ERROR, /* generic error / not accepting mission commands at all right now | */
        MISSION_RESULT_UNSUPPORTED_FRAME=MAV_MISSION_RESULT::MAV_MISSION_UNSUPPORTED_FRAME, /* coordinate frame is not supported | */
        MISSION_RESULT_UNSUPPORTED=MAV_MISSION_RESULT::MAV_MISSION_UNSUPPORTED, /* command is not supported | */
        MISSION_RESULT_NO_SPACE=MAV_MISSION_RESULT::MAV_MISSION_NO_SPACE, /* mission item exceeds storage space | */
        MISSION_RESULT_INVALID=MAV_MISSION_RESULT::MAV_MISSION_INVALID, /* one of the parameters has an invalid value | */
        MISSION_RESULT_INVALID_SEQUENCE=MAV_MISSION_RESULT::MAV_MISSION_INVALID_SEQUENCE, /* received waypoint out of sequence | */
        MISSION_RESULT_DENIED=MAV_MISSION_RESULT::MAV_MISSION_DENIED, /* not accepting any mission commands from this communication partner | */
        MISSION_RESULT_DOES_NOT_EXIST=MAV_MISSION_RESULT::MAV_MISSION_DOES_NOT_EXIST, /* the requested mission with the associated key does not exist. | */
        MISSION_RESULT_RESULT_ENUM_END=MAV_MISSION_RESULT::MAV_MISSION_RESULT_ENUM_END
    };

public:
    MissionACK(const int &systemID, const MISSION_RESULT &ack, const MissionKey &key, const MISSIONSTATE &newState);

public:
    int getSystemID() const{
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

    int m_SystemID;

    MISSION_RESULT result;

    MissionKey refKey;

    MISSIONSTATE newState;
};

} //end of namespace MissionItem
#endif // MISSION_ACK_H
