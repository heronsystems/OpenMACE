#ifndef TOPIC_COMPONENT_VOID_H
#define TOPIC_COMPONENT_VOID_H


#include "data/i_topic_component_data_object.h"

namespace Data {

namespace TopicComponents
{


extern const char TopicComponts_Void_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_Void_structure;

class Void : public Data::NamedTopicComponentDataObject<TopicComponts_Void_name, &TopicComponts_Void_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};


}

} // BaseTopics

#endif // TOPIC_COMPONENT_VOID_H
