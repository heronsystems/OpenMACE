#ifndef MISSION_ITEM_ACHIEVED_H
#define MISSION_ITEM_ACHIEVED_H

#include <mavlink.h>

#include "mission_key.h"

namespace MissionItem {

class MissionItemAchieved
{
public:
    MissionItemAchieved();
    MissionItemAchieved(const MissionKey &missionKey, const unsigned int &index);
    MissionItemAchieved(const mavlink_mission_item_reached_t &obj);

public:
    void setMissionKey(const MissionKey &missionKey){
        this->key = missionKey;
    }

    MissionKey getMissionKey() const{
        return key;
    }

    void setMissionAchievedIndex(const unsigned int &index){
        this->indexAchieved = index;
    }

    unsigned int getMissionAchievedIndex() const{
        return indexAchieved;
    }


    mavlink_mission_item_reached_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

    void operator = (const MissionItemAchieved &rhs)
    {
        this->key = rhs.key;
        this->indexAchieved = rhs.indexAchieved;
    }

    bool operator == (const MissionItemAchieved &rhs) {
        if(this->key != rhs.key){
            return false;
        }
        if(this->indexAchieved != rhs.indexAchieved){
            return false;
        }
        return true;
    }

    bool operator != (const MissionItemAchieved &rhs) {
        return !(*this == rhs);
    }

protected:
    MissionKey key;
    unsigned int indexAchieved;
};

} //end of namepsace Missionitem
#endif // MISSION_ITEM_ACHIEVED_H
