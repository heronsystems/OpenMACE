#include "occupancy_2d_grid_topic.h"

namespace MapItemTopics {

const char Occupancy2DGridTopic_name[] = "2D Occupancy Grid";
const MaceCore::TopicComponentStructure Occupancy2DGridTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::shared_ptr<Data2DGrid<OccupiedResult>>>("grid");
    return structure;
}();

MaceCore::TopicDatagram Occupancy2DGrid_Topic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::shared_ptr<Data2DGrid<OccupiedResult>>>("grid", occupancyMap);
    return datagram;
}

void Occupancy2DGrid_Topic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    occupancyMap = datagram.GetTerminal<std::shared_ptr<Data2DGrid<OccupiedResult>>>("grid");
}

} //end of namespace MapItemTopics
