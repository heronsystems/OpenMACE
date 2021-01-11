#include "command_item_topic_ack.h"

namespace CommandTopic {

const char DataCommandItemTopicACK_name[] = "Vehicle Command ACK";
const MaceCore::TopicComponentStructure DataCommandItemTopicACK_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MAV_CMD>("command");
    structure.AddTerminal<MAV_CMD_ACK>("code");
    structure.AddTerminal<int>("origin");
    structure.AddTerminal<int>("target");
    return structure;
}();

MaceCore::TopicDatagram CommandItemTopic_ACK::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MAV_CMD>("command", cmd);
    datagram.AddTerminal<MAV_CMD_ACK>("code", code);
    datagram.AddTerminal<int>("origin", originatingSystem);
    datagram.AddTerminal<int>("target", targetSystem);
    return datagram;
}

void CommandItemTopic_ACK::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    cmd = datagram.GetTerminal<MAV_CMD>("command");
    code = datagram.GetTerminal<MAV_CMD_ACK>("code");
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
