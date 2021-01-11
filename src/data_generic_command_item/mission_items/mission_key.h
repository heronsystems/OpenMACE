#ifndef MISSION_KEY_H
#define MISSION_KEY_H

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "typedef_mission_types.h"
#include "mission_state.h"

namespace MissionItem {

class MissionKey
{
public:
    MissionKey();

    MissionKey(const unsigned int &systemID, const unsigned int &creatorID, const unsigned int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &missionState = MISSIONSTATE::UNKNWON);

    MissionKey(const MissionKey &obj);

public:
    unsigned int m_systemID;

    unsigned int m_creatorID;

    //!
    //! \brief m_missionID This descriptor is a unique identifier for the vehicle to reference to
    //! the transmitter of the information in the exchange. This should help handle ack events
    //! to stations that may/may not have lost sync with vehicle. More robust methods should be
    //! investigated in the future such as timestamping etc.
    //!
    uint64_t m_missionID;

    MISSIONTYPE m_missionType;

    MISSIONSTATE m_missionState;

    MissionKey& operator =(const MissionKey &rhs);

    bool operator< (const MissionKey &rhs) const;

    bool operator== (const MissionKey &rhs) const;

    bool operator!= (const MissionKey &rhs) const;

    friend std::ostream& operator<<(std::ostream& os, const MissionKey& t);

    friend class MissionKeyHasher;
};

} //end of namespace Data

namespace std
{
template <>
struct hash<MissionItem::MissionKey>
{
    std::size_t operator()(const MissionItem::MissionKey& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

        return ((hash<int>()(k.m_systemID))
                 ^ (hash<int>()(k.m_creatorID))
                 ^ (hash<int>()(k.m_missionID))
                 ^ (hash<int>()((int)k.m_missionType))
                 ^ (hash<int>()((int)k.m_missionState)));
    }
};
}

#endif // MISSION_KEY_H
