#ifndef MISSION_ITEM_CURRENT_TOPIC_H
#define MISSION_ITEM_CURRENT_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data_generic_command_item/mission_items/mission_item_current.h"

namespace MissionTopic{

extern const char MissionItemCurrentTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemCurrentTopic_structure;

class MissionItemCurrentTopic : public MissionItem::MissionItemCurrent, public Data::NamedTopicComponentDataObject<MissionItemCurrentTopic_name, &MissionItemCurrentTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemCurrentTopic();
    MissionItemCurrentTopic(const MissionItem::MissionItemCurrent &currentItem);
    MissionItemCurrentTopic(const MissionItemCurrentTopic &copyObj);
};

}

#endif // MISSION_ITEM_CURRENT_TOPIC_H
