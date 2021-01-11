#ifndef MISSION_ITEM_CURRENT_H
#define MISSION_ITEM_CURRENT_H

#include <mavlink.h>

#include "mission_key.h"

namespace MissionItem {

class MissionItemCurrent
{
public:
    MissionItemCurrent();
    MissionItemCurrent(const MissionKey &missionKey, const unsigned int &index);
    MissionItemCurrent(const mavlink_mission_current_t &obj);

public:
    void setMissionKey(const MissionKey &missionKey){
        this->key = missionKey;
    }

    MissionKey getMissionKey() const{
        return key;
    }

    void setMissionCurrentIndex(const unsigned int &index){
        this->indexCurrent = index;
    }

    unsigned int getMissionCurrentIndex() const{
        return indexCurrent;
    }

    mavlink_mission_current_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

    void operator = (const MissionItemCurrent &rhs)
    {
        this->key = rhs.key;
        this->indexCurrent = rhs.indexCurrent;
    }

    bool operator == (const MissionItemCurrent &rhs) {
        if(this->key != rhs.key){
            return false;
        }
        if(this->indexCurrent != rhs.indexCurrent){
            return false;
        }
        return true;
    }

    bool operator != (const MissionItemCurrent &rhs) {
        return !(*this == rhs);
    }

protected:
    MissionKey key;
    unsigned int indexCurrent = 0;
};

} //end of namepsace MissionItem
#endif // MISSION_ITEM_CURRENT_H
