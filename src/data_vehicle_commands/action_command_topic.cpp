#include "action_command_topic.h"

namespace DataVehicleCommands {

const char ActionItemTopic_Name[] = "ActionItemTopic";

const MaceCore::TopicComponentStructure ActionItemTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<ActionCommandTypes>("actionItemType");
    structure.AddTerminal<std::shared_ptr<AbstractActionCommand>>("actionItem");
    return structure;
}();

MaceCore::TopicDatagram ActionCommandTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<ActionCommandTypes>("actionItemType", m_ActionItemType);
    datagram.AddTerminal<std::shared_ptr<AbstractActionCommand>>("actionItem", m_ActionCommand);
    return datagram;
}

void ActionCommandTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_ActionItemType = datagram.GetTerminal<ActionCommandTypes>("actionItemType");
    m_ActionCommand = datagram.GetTerminal<std::shared_ptr<AbstractActionCommand>>("actionItem");
}

void ActionCommandTopic::setActionItem(const std::shared_ptr<AbstractActionCommand> &actionItem)
{
    m_ActionItemType = actionItem->getActionItemType();
    m_ActionCommand = actionItem;
}

std::shared_ptr<AbstractActionCommand> ActionCommandTopic::getActionItem() const
{
    return m_ActionCommand;
}

ActionCommandTypes ActionCommandTopic::getActionItemType() const
{
    return m_ActionItemType;
}


} //end of namespace DataVehicleCommands
