#include "topic_component_string.h"

#include "common/common.h"

namespace Data {

namespace TopicComponents
{


const char TopicComponts_String_name[] = "void";
const MaceCore::TopicComponentStructure TopicComponts_String_structure = []{
    MaceCore::TopicComponentStructure structure;

    structure.AddTerminal<std::string>("string");

    return structure;
}();


MaceCore::TopicDatagram String::GenerateDatagram() const
{
    MaceCore::TopicDatagram diagram;
    diagram.AddTerminal("string", m_Str);
    return diagram;
}


void String::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_Str = datagram.GetTerminal<std::string>("string");
}

String::String() :
    m_Str("")
{

}

String::String(const std::string str) :
    m_Str(str)
{

}


} // BaseTopics

}
