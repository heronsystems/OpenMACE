#include "state_local_position_ex_topic.h"

namespace DataStateTopic{

const char LocalPositionTopicEx_name[] = "localPositionEx";
const MaceCore::TopicComponentStructure LocalPositionTopicEx_structure = [](){
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    structure.AddTerminal<double>("heading");
    return structure;
}();



MaceCore::TopicDatagram StateLocalPositionExTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    datagram.AddTerminal<double>("z", z);
    datagram.AddTerminal<double>("heading", heading);
    return datagram;
}


void StateLocalPositionExTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
    heading = datagram.GetTerminal<double>("heading");
}

StateLocalPositionExTopic::StateLocalPositionExTopic()
    :DataState::StateLocalPositionEx()
{

}

StateLocalPositionExTopic::StateLocalPositionExTopic(const DataState::StateLocalPositionEx &copyObj):
    DataState::StateLocalPositionEx(copyObj)
{

}

} //end of namespace DataStateTopic
