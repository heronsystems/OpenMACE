#include "line_2DC_topic.h"
namespace mace {
namespace geometryTopic{

const char Line_2DC_Topic_name[] = "Line 2DC";
const MaceCore::TopicComponentStructure Line_2DC_Topic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<geometry::Line_2DC>("lineItem");
    return structure;
}();

MaceCore::TopicDatagram Line_2DC_Topic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<geometry::Line_2DC>("lineItem", line);
    return datagram;
}

void Line_2DC_Topic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    line = datagram.GetTerminal<geometry::Line_2DC>("lineItem");
}

} //end of namespace geometryTopic
} //end of namespace mace
