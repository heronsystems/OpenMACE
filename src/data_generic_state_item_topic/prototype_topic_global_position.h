#ifndef PROTOTYPE_TOPIC_GLOBAL_POSITION_H
#define PROTOTYPE_TOPIC_GLOBAL_POSITION_H

#include "mace_core/topic.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_global_position.h"

namespace DataStateTopic {

extern const MaceCore::TopicComponentStructure PrototypeTopicGlobalPosition_structure;

class PrototypeTopicGlobalPosition : public DataState::StateGlobalPosition
{
public:
    PrototypeTopicGlobalPosition();

    PrototypeTopicGlobalPosition(const DataState::StateGlobalPosition &copyObj);

    virtual MaceCore::TopicDatagram Datagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};

}

#endif // PROTOTYPE_TOPIC_GLOBAL_POSITION_H
