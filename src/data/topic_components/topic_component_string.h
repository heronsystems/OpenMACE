#ifndef TOPIC_COMPONENT_STRING_H
#define TOPIC_COMPONENT_STRING_H


#include "data/i_topic_component_data_object.h"

namespace Data {

namespace TopicComponents
{


extern const char TopicComponts_String_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_String_structure;

class String : public Data::NamedTopicComponentDataObject<TopicComponts_String_name, &TopicComponts_String_structure>
{
private:

    std::string m_Str;

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    String();

    String(const std::string str);

    std::string Str() const
    {
        return m_Str;
    }
};


}

} // BaseTopics

#endif // TOPIC_COMPONENT_STRING_H
