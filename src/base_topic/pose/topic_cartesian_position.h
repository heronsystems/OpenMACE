#ifndef TOPIC_CARTESIAN_POSITION_H
#define TOPIC_CARTESIAN_POSITION_H

#include "data/i_topic_component_data_object.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

namespace pose {
namespace BaseTopic {

extern const char TopicName_CartesianPosition[];

extern const MaceCore::TopicComponentStructure Structure_CartesianPosition;

class Topic_CartesianPosition : public Data::NamedTopicComponentDataObject<TopicName_CartesianPosition, &Structure_CartesianPosition>
{

public:

    Topic_CartesianPosition();

    Topic_CartesianPosition(const Topic_CartesianPosition &copyObj);

    Topic_CartesianPosition(const mace::pose::Abstract_CartesianPosition *posObj);

    virtual ~Topic_CartesianPosition()
    {
        delete positionObj;
    }

public:
    MaceCore::TopicDatagram GenerateDatagram() const override;

    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

    mace::pose::Abstract_CartesianPosition* getPositionObj() const;

private:
    mace::pose::Abstract_CartesianPosition* positionObj;
};

} //end of namespace BaseTopic
} //end of namespace pose

#endif // TOPIC_CARTESIAN_POSITION_H
