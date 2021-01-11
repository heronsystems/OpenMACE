#include "vehicle_path_linear_topic.h"

namespace mace {
namespace topic{

const char Vehicle_Linear_Path_Topic_name[] = "Vehicle_Path_Linear";
const MaceCore::TopicComponentStructure Vehicle_Linear_Path_Topic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<VehiclePath_Linear>("path");
    return structure;
}();

MaceCore::TopicDatagram Vehicle_Path_Linear_Topic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<VehiclePath_Linear>("path", m_Path);
    return datagram;
}

void Vehicle_Path_Linear_Topic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_Path = datagram.GetTerminal<VehiclePath_Linear>("path");
}

} //end of namespace geometryTopic
} //end of namespace mace
