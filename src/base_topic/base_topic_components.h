#ifndef BASE_TOPIC_COMPONENTS_H
#define BASE_TOPIC_COMPONENTS_H

//#define BASE_GEOMETRY_TOPICS mace::geometryTopic::Line_2DC_Topic

#include "pose/topic_cartesian_position.h"
#include "pose/topic_geodetic_position.h"
#define BASE_POSE_TOPICS BaseTopic::Topic_CartesianPosition, BaseTopic::Topic_GeodeticPosition

#endif // BASE_TOPIC_COMPONENTS_H
