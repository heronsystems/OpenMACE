#include "state_global_position_topic.h"

namespace DataStateTopic{

const char GlobalPositionTopic_name[] = "globalPosition";
const MaceCore::TopicComponentStructure GlobalPositionTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    structure.AddTerminal<double>("latitude");
    structure.AddTerminal<double>("longitude");
    structure.AddTerminal<double>("altitude");
    return structure;
}();

MaceCore::TopicDatagram StateGlobalPositionTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("latitude", x);
    datagram.AddTerminal<double>("longitude", y);
    datagram.AddTerminal<double>("altitude", z);
    return datagram;
}

void StateGlobalPositionTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    x = datagram.GetTerminal<double>("latitude");
    y = datagram.GetTerminal<double>("longitude");
    z = datagram.GetTerminal<double>("altitude");
}

StateGlobalPositionTopic::StateGlobalPositionTopic()
    :DataState::StateGlobalPosition()
{

}

StateGlobalPositionTopic::StateGlobalPositionTopic(const StateGlobalPositionTopic &copyObj):
    DataState::StateGlobalPosition(copyObj)
{

}

StateGlobalPositionTopic::StateGlobalPositionTopic(const DataState::StateGlobalPosition &posObj):
    DataState::StateGlobalPosition(posObj)
{

}
} //end of namespace DataStateTopic
