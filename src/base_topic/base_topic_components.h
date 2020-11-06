#ifndef BASE_TOPIC_COMPONENTS_H
#define BASE_TOPIC_COMPONENTS_H

#define BASE_POSE_TOPICS mace::pose_topics::Topic_CartesianPosition, mace::pose_topics::Topic_GeodeticPosition, mace::pose_topics::Topic_CartesianVelocity, mace::pose_topics::Topic_AgentOrientation,mace::pose_topics::Topic_RotationalVelocity

#define BASE_GEOMETRY_TOPICS mace::pose_topics::Topic_CartesianPosition

#include "pose/topic_agent_orientation.h"
#include "pose/topic_cartesian_position.h"
#include "pose/topic_cartesian_velocity.h"
#include "pose/topic_geodetic_position.h"
#include "pose/topic_rotational_velocity.h"

#define VEHICLE_MEASUREMENT_TOPICS mace::measurement_topics::Topic_AirSpeed, mace::measurement_topics::Topic_GroundSpeed

#include "measurements/topic_speed.h"

#endif // BASE_TOPIC_COMPONENTS_H
