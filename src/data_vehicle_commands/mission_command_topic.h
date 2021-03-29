#ifndef MISSION_COMMAND_TOPIC_H
#define MISSION_COMMAND_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "abstract_mission_item.h"

namespace DataVehicleCommands
{

extern const char MissionCommandTopic_Name[];
extern const MaceCore::TopicComponentStructure MissionCommandTopic_Structure;

class MissionCommandTopic : public Data::NamedTopicComponentDataObject<MissionCommandTopic_Name, &MissionCommandTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setMissionItem(const std::shared_ptr<AbstractMissionItem> &actionItem);

    std::shared_ptr<AbstractMissionItem> getActionItem() const;

    MissionItemTypes getActionItemType() const;

private:
    MissionItemTypes m_MissionCommandType;
    std::shared_ptr<AbstractMissionItem> m_MissionItem;
};
} //end of namespace DataVehicleCommands
#endif // MISSION_COMMAND_TOPIC_H
