#include "command_item_topic.h"

namespace CommandTopic{

const char CommandItemTopic_name[] = "Command Item";
const MaceCore::TopicComponentStructure CommandItemTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::shared_ptr<command_item::AbstractCommandItem>>("commandItem");
    return structure;
}();

MaceCore::TopicDatagram CommandItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::shared_ptr<command_item::AbstractCommandItem>>("commandItem", commandItem);
    return datagram;
}

void CommandItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    commandItem = datagram.GetTerminal<std::shared_ptr<command_item::AbstractCommandItem>>("commandItem");
}

} //end of namespace MissionTopic
