#include "command_topic_takeoff.h"

namespace DataCommandTopic {

const char DataCommandTopicTakeoff_name[] = "command_takeoff";
const MaceCore::TopicComponentStructure DataCommandTopicTakeoff_structure = []{
    return DataStateTopic::PrototypeTopicGlobalPosition_structure;
}();

MaceCore::TopicDatagram DataCommandTopic_Takeoff::GenerateDatagram() const {
    return DataStateTopic::PrototypeTopicGlobalPosition::Datagram();
}

void DataCommandTopic_Takeoff::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    DataStateTopic::PrototypeTopicGlobalPosition::CreateFromDatagram(datagram);
}

DataCommandTopic_Takeoff::DataCommandTopic_Takeoff()
    : DataStateTopic::PrototypeTopicGlobalPosition()
{

}

DataCommandTopic_Takeoff::DataCommandTopic_Takeoff(const DataState::Base3DPosition &copyObj):
    DataStateTopic::PrototypeTopicGlobalPosition()
{
    throw std::runtime_error("Not Implimented");
}

} //end of namespace DataCommandTopic
