#ifndef EMPTY_MAVLINK_H
#define EMPTY_MAVLINK_H

#include "data/i_topic_component_data_object.h"

namespace DataMAVLINK {

extern const char EmptyMAVLINK_name[];
extern const MaceCore::TopicComponentStructure EmptyMAVLINK_structure;

class EmptyMAVLINK : public Data::NamedTopicComponentDataObject<EmptyMAVLINK_name, &EmptyMAVLINK_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    EmptyMAVLINK();

};

} //end of namespace DataMAVLINK

#endif // EMPTY_MAVLINK_H
