#ifndef TOPIC_COMPONENT_MISSION_KEY_H
#define TOPIC_COMPONENT_MISSION_KEY_H

#include "data/i_topic_component_data_object.h"

#include "mission_type.h"
#include "mission_state.h"

namespace Data {

namespace TopicComponents
{


extern const char TopicComponts_MissionKey_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_MissionKey_structure;

class MissionKey : public Data::NamedTopicComponentDataObject<TopicComponts_MissionKey_name, &TopicComponts_MissionKey_structure>
{
public:

    int m_systemID;
    int m_creatorID;

    //!
    //! \brief m_missionID This descriptor is a unique identifier for the vehicle to reference to
    //! the transmitter of the information in the exchange. This should help handle ack events
    //! to stations that may/may not have lost sync with vehicle. More robust methods should be
    //! investigated in the future such as timestamping etc.
    //!
    uint64_t m_missionID;

    Data::MISSIONTYPE m_missionType;

    Data::MISSIONSTATE m_missionState;

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    MissionKey();
    MissionKey(const int &systemID, const int &creatorID, const int &missionID, const MISSIONTYPE &missionType);
    MissionKey(const int &systemID, const int &creatorID, const int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &missionState);
    MissionKey(const MissionKey &obj);

    MissionKey& operator =(const MissionKey &rhs);

    bool operator< (const MissionKey &rhs) const;

    bool operator== (const MissionKey &rhs) const;

    bool operator!= (const MissionKey &rhs) const;

    friend std::ostream& operator<<(std::ostream& os, const MissionKey& t);
};


}

} // BaseTopics


namespace std
{
template <>
struct hash<Data::TopicComponents::MissionKey>
{
    std::size_t operator()(const Data::TopicComponents::MissionKey& k) const
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

#endif // TOPIC_COMPONENT_MISSION_KEY_H
