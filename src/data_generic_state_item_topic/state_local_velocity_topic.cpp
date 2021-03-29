#include "state_local_velocity_topic.h"

namespace DataStateTopic{

const char LocalVelocityTopic_name[] = "localVelocity";
const MaceCore::TopicComponentStructure LocalVelocityTopic_structure = [](){
        MaceCore::TopicComponentStructure structure;
        structure.AddTerminal<double>("x");
        structure.AddTerminal<double>("y");
        structure.AddTerminal<double>("z");
        return structure;
    }();

MaceCore::TopicDatagram StateLocalVelocityTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    return datagram;
}

void StateLocalVelocityTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
}

StateLocalVelocityTopic::StateLocalVelocityTopic()
    :DataState::StateLocalVelocity()
{

}

StateLocalVelocityTopic::StateLocalVelocityTopic(const DataState::StateLocalVelocity &copyObj):
    DataState::StateLocalVelocity(copyObj)
{

}

} //end of namespace DataStateTopic
