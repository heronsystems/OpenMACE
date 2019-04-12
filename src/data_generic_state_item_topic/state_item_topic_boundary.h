#ifndef STATE_ITEM_TOPIC_BOUNDARYH_H
#define STATE_ITEM_TOPIC_BOUNDARYH_H

#include "data_generic_state_item/state_item_boundary.h"
#include "data/i_topic_component_data_object.h"

namespace DataStateTopic {

extern const char StateItemTopicBoundary_name[];
extern const MaceCore::TopicComponentStructure StateItemTopicBoundary_structure;

class StateItemTopic_Boundary : public DataState::StateItem_Boundary, public Data::NamedTopicComponentDataObject<StateItemTopicBoundary_name, &StateItemTopicBoundary_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateItemTopic_Boundary();
    StateItemTopic_Boundary(const DataState::StateItem_Boundary &copyObj);

public:
    std::vector<DataState::StateGlobalPosition> getEnvironmentVertices() const;
    void setEnvironmentVertices(const std::vector<DataState::StateGlobalPosition> &vertices);

private:
    std::vector<DataState::StateGlobalPosition> boundaryVerts;

};

} //end of namespace DataGenericItemTopic

#endif // STATE_ITEM_TOPIC_BOUNDARYH_H
