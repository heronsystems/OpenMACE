#include "state_item_topic_boundary.h"

namespace DataStateTopic {

const char StateItemTopicBoundary_name[] = "environmentBoundary";
const MaceCore::TopicComponentStructure StateItemTopicBoundary_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::vector<DataState::StateGlobalPosition> >("boundaryVertices");
    return structure;
}();


MaceCore::TopicDatagram StateItemTopic_Boundary::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::vector<DataState::StateGlobalPosition> >("boundaryVertices", boundaryVerts);
    return datagram;
}

void StateItemTopic_Boundary::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    boundaryVerts = datagram.GetTerminal<std::vector<DataState::StateGlobalPosition> >("boundaryVertices");
}

StateItemTopic_Boundary::StateItemTopic_Boundary()
    : DataState::StateItem_Boundary()
{
}

StateItemTopic_Boundary::StateItemTopic_Boundary(const DataState::StateItem_Boundary &copyObj)
    : DataState::StateItem_Boundary(copyObj)
{
}

std::vector<DataState::StateGlobalPosition> StateItemTopic_Boundary::getEnvironmentVertices() const
{
    return(boundaryVerts);
}

void StateItemTopic_Boundary::setEnvironmentVertices(const std::vector<DataState::StateGlobalPosition> &vertices)
{
    boundaryVerts = vertices;
}

} //end of namespace DataGenericItemTopic
