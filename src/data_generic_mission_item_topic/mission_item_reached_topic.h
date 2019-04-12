#ifndef MISSION_ITEM_REACHED_TOPIC_H
#define MISSION_ITEM_REACHED_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data_generic_command_item/mission_items/mission_item_achieved.h"
namespace MissionTopic{

extern const char MissionItemReachedTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemReachedTopic_structure;

class MissionItemReachedTopic : public MissionItem::MissionItemAchieved, public Data::NamedTopicComponentDataObject<MissionItemReachedTopic_name, &MissionItemReachedTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemReachedTopic();
    MissionItemReachedTopic(const MissionItem::MissionItemAchieved &achievedItem);
    MissionItemReachedTopic(const MissionItemReachedTopic &copyObj);
};

}

#endif // MISSION_ITEM_REACHED_TOPIC_H
