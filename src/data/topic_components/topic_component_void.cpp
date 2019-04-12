#include "topic_component_void.h"

#include "common/common.h"

namespace Data {

namespace TopicComponents
{


const char TopicComponts_Void_name[] = "void";
const MaceCore::TopicComponentStructure TopicComponts_Void_structure = []{
    return MaceCore::TopicComponentStructure();
}();


MaceCore::TopicDatagram Void::GenerateDatagram() const
{
    return MaceCore::TopicDatagram();
}


void Void::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    UNUSED(datagram);
}




} // BaseTopics

}
