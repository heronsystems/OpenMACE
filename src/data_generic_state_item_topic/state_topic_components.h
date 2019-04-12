#ifndef STATE_TOPIC_COMPONENTS_H
#define STATE_TOPIC_COMPONENTS_H

#define DATA_STATE_GENERIC_TOPICS DataStateTopic::StateAirspeedTopic, DataStateTopic::StateAttitudeTopic,\
    DataStateTopic::StateGlobalPositionTopic,DataStateTopic::StateGlobalPositionExTopic,\
    DataStateTopic::StateGlobalVelocityTopic,DataStateTopic::StateLocalPositionTopic,\
    DataStateTopic::StateLocalPositionExTopic,DataStateTopic::StateLocalVelocityTopic, \
    DataStateTopic::StateItemTopic_Boundary

#include "state_airspeed_topic.h"
#include "state_attitude_topic.h"

#include "state_global_position_topic.h"
#include "state_global_position_ex_topic.h"

#include "state_local_position_topic.h"
#include "state_local_position_ex_topic.h"

#include "state_global_velocity_topic.h"
#include "state_local_velocity_topic.h"

#include "state_item_topic_boundary.h"

#endif // STATE_TOPIC_COMPONENTS_H
