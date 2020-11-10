/*!
  * @file virtual_target_controller.h
  *
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *
  * @section DESCRIPTION
  *
  * @date
  *     Feb 2020
  *
  * @copyright
  *     File and its related contents are subjected to a proprietary software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#ifndef VIRTUAL_TARGET_CONTROLLER_H
#define VIRTUAL_TARGET_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <vector>
#include "common.h"

#include "agent_parameters.h"
#include "data_get_set_notifier.h"
#include "ros_message_definitions.h"
#include "thread_manager.h"

#include "pose/pose_components.h"
#include "vehicle/vehicle_state.h"

typedef void(*CallbackFunctionPtr_VehicleTargetTransmission)(void*, const pose::GeodeticPosition_3D&, const pose::GeodeticPosition_3D&);

class VirtualTargetController
{

public:
    enum TimeoutMode
    {
        NORMAL,
        LEVELING,
        ABORT
    };

private:
    struct TimeoutDataStruct
    {
        TimeoutMode mode;
        int timeout;
        void* functionTarget;
    };

public:
    VirtualTargetController(const std::string &agentID, const unsigned int &timeout = 500);

    virtual ~VirtualTargetController() = default;

public:
    void connectVehicleTargetCallback(CallbackFunctionPtr_VehicleTargetTransmission cb, void *p)
    {
        m_CBVehicleTarget = cb;
        m_FunctionTarget = p;
    }

    void callGeodeticTargetCallback(const pose::GeodeticPosition_3D &projection, const pose::GeodeticPosition_3D &actual)
    {
        if(m_CBVehicleTarget != nullptr)
            m_CBVehicleTarget(m_FunctionTarget,projection, actual);
    }

private:
    CallbackFunctionPtr_VehicleTargetTransmission m_CBVehicleTarget;
    void *m_FunctionTarget;

public:
    void updateAgentParams(const AgentParams &params); 

    void updateOriginReference(const pose::GeodeticPosition_3D &position);

    void updateAgentState(const VehicleState_Cartesian3D &state);

    void registerCurrentTarget(const pose::CartesianPosition_3D &commandTarget, const TimeoutMode &mode = TimeoutMode::NORMAL);

private:
    void updateTargetProjection();

    void publishVehicleTarget(const pose::GeodeticPosition_3D &target);

private: 
    AgentParams m_AgentParams;
    
    DataGetSetNotifier<pose::GeodeticPosition_3D> m_Origin;
    DataGetSetNotifier<pose::CartesianPosition_3D> m_CurrentTarget;
    DataGetSetNotifier<VehicleState_Cartesian3D> m_VehicleState;

    DataGetSetNotifier<pose::GeodeticPosition_3D> m_VirtualTarget_Geo;
};

#endif // VIRTUAL_TARGET_CONTROLLER_H
