#include "mission_home_topic.h"

namespace MissionTopic{

const char MissionHomeTopic_name[] = "Current Vehicle Home";
const MaceCore::TopicComponentStructure MissionHomeTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<command_item::AbstractCommandItem>("homeItem");
    return structure;
}();

MaceCore::TopicDatagram MissionHomeTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<command_item::SpatialHome>("homeItem",item);
    return datagram;
}

void MissionHomeTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    item = datagram.GetTerminal<command_item::SpatialHome>("homeItem");
}



} //end of namespace MissionTopic
