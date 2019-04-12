#ifndef MISSION_KEY_CHANGE_H
#define MISSION_KEY_CHANGE_H

#include "mission_key.h"

namespace MissionItem {

class MissionKeyChange
{
public:
    MissionKeyChange(const MissionKey &oldK, const MissionKey &newK);

    bool operator == (const MissionKeyChange &rhs) const{
        if(this->oldKey != rhs.oldKey){
            return false;
        }
        if(this->newKey != rhs.newKey){
            return false;
        }
        return true;
    }

    bool operator != (const MissionKeyChange &rhs) {
        return !(*this == rhs);
    }

public:
    MissionKey oldKey;
    MissionKey newKey;
};

} //end of namespace Data
#endif // MISSION_KEY_CHANGE_H
