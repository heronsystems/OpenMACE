#include "state_attitude_topic.h"

namespace DataStateTopic {

const char AttitudeTopic_name[] = "attitude";
const MaceCore::TopicComponentStructure AttitudeTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("roll");
    structure.AddTerminal<double>("rollRate");
    structure.AddTerminal<double>("pitch");
    structure.AddTerminal<double>("pitchRate");
    structure.AddTerminal<double>("yaw");
    structure.AddTerminal<double>("yawRate");

    return structure;
}();

MaceCore::TopicDatagram StateAttitudeTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("roll", roll);
    datagram.AddTerminal<double>("rollRate", rollRate);
    datagram.AddTerminal<double>("pitch", pitch);
    datagram.AddTerminal<double>("pitchRate", pitchRate);
    datagram.AddTerminal<double>("yaw", yaw);
    datagram.AddTerminal<double>("yawRate", yawRate);
    return datagram;
}

void StateAttitudeTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    roll = datagram.GetTerminal<double>("roll");
    rollRate = datagram.GetTerminal<double>("rollRate");
    pitch = datagram.GetTerminal<double>("pitch");
    pitchRate = datagram.GetTerminal<double>("pitchRate");
    yaw = datagram.GetTerminal<double>("yaw");
    yawRate = datagram.GetTerminal<double>("yawRate");
}

StateAttitudeTopic::StateAttitudeTopic()
    :DataState::StateAttitude()
{

}

StateAttitudeTopic::StateAttitudeTopic(const DataState::StateAttitude &copyObj):
    DataState::StateAttitude(copyObj)
{

}

} //end of namespace DataStateTopic
