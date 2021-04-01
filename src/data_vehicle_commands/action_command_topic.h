#ifndef ACTION_COMMAND_TOPIC_H
#define ACTION_COMMAND_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "abstract_action_command.h"

namespace DataVehicleCommands
{

extern const char ActionItemTopic_Name[];
extern const MaceCore::TopicComponentStructure ActionItemTopic_Structure;

class ActionCommandTopic : public Data::NamedTopicComponentDataObject<ActionItemTopic_Name, &ActionItemTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setActionItem(const std::shared_ptr<AbstractActionCommand> &actionItem);

    std::shared_ptr<AbstractActionCommand> getActionItem() const;

    ActionCommandTypes getActionItemType() const;

private:
    ActionCommandTypes m_ActionItemType;
    std::shared_ptr<AbstractActionCommand> m_ActionCommand;
};
} //end of namespace DataVehicleCommands
#endif // ACTION_COMMAND_TOPIC_H
