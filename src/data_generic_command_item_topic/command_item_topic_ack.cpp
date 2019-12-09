#include "command_item_topic_ack.h"

namespace CommandTopic {

const char DataCommandItemTopicACK_name[] = "Vehicle Command ACK";
const MaceCore::TopicComponentStructure DataCommandItemTopicACK_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<command_item::COMMANDTYPE>("command");
    structure.AddTerminal<Data::CommandACKType>("code");
    structure.AddTerminal<int>("origin");
    structure.AddTerminal<int>("target");
    return structure;
}();

MaceCore::TopicDatagram CommandItemTopic_ACK::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<COMMANDTYPE>("command", cmd);
    datagram.AddTerminal<Data::CommandACKType>("code", code);
    datagram.AddTerminal<int>("origin", originatingSystem);
    datagram.AddTerminal<int>("target", targetSystem);
    return datagram;
}

void CommandItemTopic_ACK::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    cmd = datagram.GetTerminal<COMMANDTYPE>("command");
    code = datagram.GetTerminal<Data::CommandACKType>("code");
    originatingSystem = datagram.GetTerminal<int>("origin");
    targetSystem = datagram.GetTerminal<int>("target");
}

CommandItemTopic_ACK::CommandItemTopic_ACK():
    command_item::CommandItemACK()
{

}

CommandItemTopic_ACK::CommandItemTopic_ACK(const command_item::CommandItemACK &copyObj):
    command_item::CommandItemACK(copyObj)
{

}

} //end of namespace DataGenericItemTopic
