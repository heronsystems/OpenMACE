#include "state_global_velocity_topic.h"

namespace DataStateTopic{

const char GlobalVelocityTopic_name[] = "globalVelocity";
const MaceCore::TopicComponentStructure GlobalVelocityTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    structure.AddTerminal<double>("heading");
    return structure;
}();




MaceCore::TopicDatagram StateGlobalVelocityTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    datagram.AddTerminal<double>("heading", heading);
    return datagram;
}


void StateGlobalVelocityTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
    heading = datagram.GetTerminal<double>("heading");
}

StateGlobalVelocityTopic::StateGlobalVelocityTopic()
    :DataState::StateGlobalVelocity()
{

}

StateGlobalVelocityTopic::StateGlobalVelocityTopic(const DataState::StateGlobalVelocity &copyObj):
    DataState::StateGlobalVelocity(copyObj)
{

}

} //end of namespace DataStateTopic

