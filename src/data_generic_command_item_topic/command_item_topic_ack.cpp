#include "command_item_topic_ack.h"

namespace CommandTopic {

const char DataCommandItemTopicACK_name[] = "Vehicle Command ACK";
const MaceCore::TopicComponentStructure DataCommandItemTopicACK_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<CommandItem::COMMANDITEM>("command");
    structure.AddTerminal<Data::CommandACKType>("code");
    structure.AddTerminal<int>("origin");
    structure.AddTerminal<int>("target");
    return structure;
}();

MaceCore::TopicDatagram CommandItemTopic_ACK::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<COMMANDITEM>("command", cmd);
    datagram.AddTerminal<Data::CommandACKType>("code", code);
    datagram.AddTerminal<int>("origin", originatingSystem);
    datagram.AddTerminal<int>("target", targetSystem);
    return datagram;
}

void CommandItemTopic_ACK::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    cmd = datagram.GetTerminal<COMMANDITEM>("command");
    code = datagram.GetTerminal<Data::CommandACKType>("code");
    originatingSystem = datagram.GetTerminal<int>("origin");
    targetSystem = datagram.GetTerminal<int>("target");
}

CommandItemTopic_ACK::CommandItemTopic_ACK():
    CommandItem::CommandItemACK()
{

}

CommandItemTopic_ACK::CommandItemTopic_ACK(const CommandItem::CommandItemACK &copyObj):
    CommandItem::CommandItemACK(copyObj)
{

}

} //end of namespace DataGenericItemTopic
