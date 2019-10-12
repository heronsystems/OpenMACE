#ifndef OCCUPANCY_2D_TOPIC_H
#define OCCUPANCY_2D_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data_2d_grid.h"
#include "occupancy_definition.h"

using namespace mace::maps;

namespace MapItemTopics {

extern const char Occupancy2DGridTopic_name[];
extern const MaceCore::TopicComponentStructure Occupancy2DGridTopic_structure;

class Occupancy2DGrid_Topic :public Data::NamedTopicComponentDataObject<Occupancy2DGridTopic_name, &Occupancy2DGridTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    Occupancy2DGrid_Topic():
        occupancyMap(nullptr)
    {

    }

    Occupancy2DGrid_Topic(const std::shared_ptr<Data2DGrid<OccupiedResult>> &map)
    {
        this->setMap(map);
    }

    void setMap(const std::shared_ptr<Data2DGrid<OccupiedResult>> &map)
    {
        this->occupancyMap = map;
    }

    std::shared_ptr<Data2DGrid<OccupiedResult>> getOccupancyMap() const
    {
        return occupancyMap;
    }

private:
    std::shared_ptr<Data2DGrid<OccupiedResult>> occupancyMap;
};

} //end of namespace MapItemTopics

#endif // OCCUPANCY_2D_TOPIC_H
