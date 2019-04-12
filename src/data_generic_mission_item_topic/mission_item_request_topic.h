#ifndef MISSION_ITEM_REQUEST_TOPIC_H
#define MISSION_ITEM_REQUEST_TOPIC_H

#include "data/i_topic_component_data_object.h"

namespace MissionTopic{

extern const char MissionItemRequestTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemRequestTopic_structure;

class MissionItemRequestTopic :public Data::NamedTopicComponentDataObject<MissionItemRequestTopic_name, &MissionItemRequestTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemRequestTopic(const int &vehicleID, const int &missionItemIndex);

public:
    int vehicleID;
    int missionItemIndex;
};

}

#endif // MISSION_ITEM_REQUEST_TOPIC_H
