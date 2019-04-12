#include "state_local_position_topic.h"

namespace DataStateTopic{

const char LocalPositionTopic_name[] = "localPosition";
const MaceCore::TopicComponentStructure LocalPositionTopic_structure = [](){
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    return structure;
}();



MaceCore::TopicDatagram StateLocalPositionTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    return datagram;
}


void StateLocalPositionTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
}

StateLocalPositionTopic::StateLocalPositionTopic()
    :DataState::StateLocalPosition()
{

}

StateLocalPositionTopic::StateLocalPositionTopic(const DataState::StateLocalPosition &copyObj):
    DataState::StateLocalPosition(copyObj)
{

}

} //end of namespace DataStateTopic
