#include "prototype_topic_global_position.h"



namespace DataStateTopic{

const MaceCore::TopicComponentStructure PrototypeTopicGlobalPosition_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    structure.AddTerminal<double>("latitude");
    structure.AddTerminal<double>("longitude");
    structure.AddTerminal<double>("altitude");
    return structure;
}();


PrototypeTopicGlobalPosition::PrototypeTopicGlobalPosition()
{

}

PrototypeTopicGlobalPosition::PrototypeTopicGlobalPosition(const DataState::StateGlobalPosition &copyObj) :
    DataState::StateGlobalPosition(copyObj)
{

}

MaceCore::TopicDatagram PrototypeTopicGlobalPosition::Datagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("latitude", x);
    datagram.AddTerminal<double>("longitude", y);
    datagram.AddTerminal<double>("altitude", z);
    return datagram;
}

void PrototypeTopicGlobalPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    x = datagram.GetTerminal<double>("latitude");
    y = datagram.GetTerminal<double>("longitude");
    z = datagram.GetTerminal<double>("altitude");
}

}
