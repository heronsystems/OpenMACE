#include "position_local.h"

namespace Data {

namespace TopicComponents
{

const char TopicComponts_LocalPosition_name[] = "position_local";
const MaceCore::TopicComponentStructure TopicComponts_LocalPosition_structure = []{
    return TopicComponentPrototypes::PositionCartesian3D_structure;
}();

} // TopicComponents

} // Data
