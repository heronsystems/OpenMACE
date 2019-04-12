#ifndef TOPIC_PROTOTYPES_POSITION_CARTESIAN_H
#define TOPIC_PROTOTYPES_POSITION_CARTESIAN_H


#include "mace_core/topic.h"

#include "../reference_cartesian.h"

namespace Data {

namespace TopicComponentPrototypes
{

extern const MaceCore::TopicComponentStructure PositionCartesian3D_structure;

class PositionCartesian3D
{
private:

    double m_x;
    double m_y;
    double m_z;
    ReferenceCartesian m_Reference;

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    PositionCartesian3D(const double &x, const double &y, const double &z, const ReferenceCartesian &ref);

    PositionCartesian3D(const PositionCartesian3D &copyObj);
};


} // TopicComponents

} // Data

#endif // TOPIC_PROTOTYPES_POSITION_CARTESIAN_H
