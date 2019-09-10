#ifndef MATLAB_LISTENER_H
#define MATLAB_LISTENER_H

#ifdef ROS_EXISTS
#include <ros/ros.h>

#include <mace_matlab_msgs/CMD_ARM.h>
#include <mace_matlab_msgs/CMD_DATUM.h>
#include <mace_matlab_msgs/CMD_LAND.h>
#include <mace_matlab_msgs/CMD_WPT.h>
#include <mace_matlab_msgs/CMD_TAKEOFF.h>
#endif

#include "base/pose/dynamics_aid.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ROS.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_sensors/components.h"
#include "data_generic_state_item/positional_aid.h"

#include "base_topic/vehicle_topics.h"


class MATLABListener
{
public:

    enum class MATLAB_CMD {
        CMD_TAKEOFF = 0,
        CMD_LAND = 1,
        CMD_ARM = 2,
        CMD_WPT = 3,
        CMD_DATUM = 4
    };

    enum class MATLAB_CMD_STATUS {
        STATUS_EXECUTING = 0,
        STATUS_COMPLETE = 1,
        STATUS_ERROR = 2
    };


public:
    MATLABListener(const MaceCore::IModuleCommandROS* ptrRef);

#ifdef ROS_EXISTS
    bool commandTakeoff(mace_matlab::CMD_TAKEOFF::Request  &req,
                        mace_matlab::CMD_TAKEOFF::Response &res);

    bool commandArm(mace_matlab::CMD_ARM::Request  &req,
                    mace_matlab::CMD_ARM::Response &res);

    bool commandLand(mace_matlab::CMD_LAND::Request  &req,
                     mace_matlab::CMD_LAND::Response &res);

    bool commandWaypoint(mace_matlab::CMD_WPT::Request  &req,
                         mace_matlab::CMD_WPT::Response &res);

    bool commandDatum(mace_matlab::CMD_DATUM::Request  &req,
                      mace_matlab::CMD_DATUM::Response &res);
#endif

private:
    //!
    //! \brief m_parent Reference to parent object
    //!
    const MaceCore::IModuleCommandROS* m_parent;

};

#endif // MATLAB_LISTENER_H
