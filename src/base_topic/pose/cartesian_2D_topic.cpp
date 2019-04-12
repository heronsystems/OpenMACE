#include "cartesian_2D_topic.h"
namespace mace {
namespace poseTopic{

const char Cartesian_2D_Topic_name[] = "Cartesian_2D";
const MaceCore::TopicComponentStructure Cartesian_2D_Topic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<pose::CartesianPosition_2D>("poseItem");
    return structure;
}();

MaceCore::TopicDatagram Cartesian_2D_Topic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<pose::CartesianPosition_2D>("poseItem", pose);
    return datagram;
}

void Cartesian_2D_Topic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    pose = datagram.GetTerminal<pose::CartesianPosition_2D>("poseItem");
}

} //end of namespace geometryTopic
} //end of namespace mace
