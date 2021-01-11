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

#include <iostream>
#include <vector>
#include "common/common.h"

#include "base/trajectory/agent_parameters.h"
#include "data/data_get_set_notifier.h"
#include "common/thread_manager.h"

#include "base/pose/pose_components.h"
#include "base/vehicle/vehicle_state.h"

using namespace mace;

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
    VirtualTargetController(const std::string &agentID = "", const unsigned int &timeout = 500);

    VirtualTargetController(const VirtualTargetController &copy) = delete;

    ~VirtualTargetController()
    {

    }

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
    void shutdownTargetController()
    {
        m_CBVehicleTarget = nullptr;
        m_FunctionTarget = nullptr;
    }

    void updateOriginReference(const pose::GeodeticPosition_3D &position);

    void updateAgentState(const VehicleState_Cartesian3D &state);

    void registerCurrentTarget(const pose::CartesianPosition_3D &commandTarget, const TimeoutMode &mode = TimeoutMode::NORMAL);

    void invalidateCurrentTarget();

private:
    void updateTargetProjection();

    void publishVehicleTarget(const pose::GeodeticPosition_3D &target);

private:     
    Data::DataGetSetNotifier<pose::GeodeticPosition_3D> m_Origin;
    Data::DataGetSetNotifier<pose::CartesianPosition_3D> m_CurrentTarget;
    Data::DataGetSetNotifier<VehicleState_Cartesian3D> m_VehicleState;
    Data::DataGetSetNotifier<pose::GeodeticPosition_3D> m_VirtualTarget_Geo;
};

#endif // VIRTUAL_TARGET_CONTROLLER_H
