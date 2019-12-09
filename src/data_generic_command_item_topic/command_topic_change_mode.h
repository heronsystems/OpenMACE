#ifndef COMMAND_TOPIC_CHANGE_MODE_H
#define COMMAND_TOPIC_CHANGE_MODE_H

#include "data/i_topic_component_data_object.h"

namespace DataCommandTopic {

extern const char DataCommandTopicChangeMode_name[];
extern const MaceCore::TopicComponentStructure DataCommandTopicChangeMode_structure;

class DataCommandTopic_ChangeMode : public std::string, public Data::NamedTopicComponentDataObject<DataCommandTopicChangeMode_name, &DataCommandTopicChangeMode_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    DataCommandTopic_ChangeMode();
    DataCommandTopic_ChangeMode(const std::string &copyObj);
};

} //end of namespace DataCommandTopic

#endif // COMMAND_TOPIC_CHANGE_MODE_H
