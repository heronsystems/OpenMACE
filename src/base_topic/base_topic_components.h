#ifndef BASE_TOPIC_COMPONENTS_H
#define BASE_TOPIC_COMPONENTS_H

#define BASE_POSE_TOPICS mace::pose_topics::Topic_CartesianPosition, mace::pose_topics::Topic_GeodeticPosition, mace::pose_topics::Topic_AgentOrientation

#define BASE_GEOMETRY_TOPICS mace::pose_topics::Topic_CartesianPosition

#include "pose/topic_agent_orientation.h"
#include "pose/topic_cartesian_position.h"
#include "pose/topic_geodetic_position.h"

#endif // BASE_TOPIC_COMPONENTS_H
