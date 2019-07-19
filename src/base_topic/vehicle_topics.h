#ifndef VEHICLE_TOPICS_H
#define VEHICLE_TOPICS_H

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "base_topic/pose/topic_cartesian_position.h"
#include "base_topic/pose/topic_cartesian_velocity.h"
#include "base_topic/pose/topic_geodetic_position.h"

#include "data/topic_components/altitude.h"
#include "data/topic_components/position_global.h"
#include "data/topic_components/position_local.h"
#include "data/topic_components/topic_component_string.h"
#include "data/topic_components/topic_component_void.h"

namespace BaseTopic {


class VehicleTopics
{
public:
    MaceCore::NonSpooledTopic<Data::TopicComponents::Void, pose::BaseTopic::Topic_GeodeticPosition> m_CommandTakeoff;
    MaceCore::NonSpooledTopic<Data::TopicComponents::Void, pose::BaseTopic::Topic_GeodeticPosition> m_CommandLand;
    MaceCore::NonSpooledTopic<Data::TopicComponents::String> m_CommandSystemMode;

public:
    VehicleTopics();
};


}

#endif // VEHICLE_TOPICS_H
