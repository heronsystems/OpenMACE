#ifndef MISSION_ITEM_TOPIC_H
#define MISSION_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/abstract_command_item.h"

namespace MissionTopic{

extern const char MissionItemTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemTopic_structure;

class MissionItemTopic :public Data::NamedTopicComponentDataObject<MissionItemTopic_name, &MissionItemTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemTopic();

    void setMissionItem(const std::shared_ptr<command_item::AbstractCommandItem> missionItem);

    std::shared_ptr<command_item::AbstractCommandItem> getMissionItem();

private:
    std::shared_ptr<command_item::AbstractCommandItem> missionItem;
};

} //end of namespace MissionTopic
#endif // MISSION_ITEM_TOPIC_H
