/*!
  * @file vehicle_object_arduplane.cpp
  *
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *     This is a part of Heron Systems MACE ecosystem.
  *
  * @section DESCRIPTION
  *
  * @date
  *     August 2020
  *
  * @copyright
  *     File and its related contents are subjected to a software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#include "vehicle_object_arduplane.h"

#include "base/math/frame_tf.h"

VehicleObject_Arduplane::VehicleObject_Arduplane(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    VehicleObject_Ardupilot(commsObj, module, mavlinkID), m_TrackingManager(nullptr), m_TargetController(nullptr)
{
    m_ArdupilotMode = new ARDUPLANEComponent_FlightMode();

    m_TrackingManager = new TrackingSimplified();

    m_TargetController = new VirtualTargetController(std::to_string(mavlinkID));

    state->m_CompleteVehicleCarState.AddNotifier(this, [this]{
        VehicleState_Cartesian3D currentState = state->m_CompleteVehicleCarState.get();
        Eigen::Vector3d dataENU = ftf::transform_frame_ned_enu(currentState.m_Position.getDataVector());
        currentState.m_Position.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_ENU);
        currentState.m_Position.updatePosition(dataENU(0),dataENU(1),dataENU(2));

        m_TrackingManager->updatedAgentState(currentState);
        m_TargetController->updateAgentState(currentState);
    });

    environment->vehicleGlobalOrigin.AddNotifier(this, [this]{
        m_TargetController->updateOriginReference(environment->vehicleGlobalOrigin.get());
    });

    m_TrackingManager->addLambda_TargetUpdate(this,[this](const pose::CartesianPosition_3D &target)
    {
        if(target.areAllPositionsValid())
            m_TargetController->registerCurrentTarget(target);
    });

}

VehicleObject_Arduplane::~VehicleObject_Arduplane()
{
    if(m_ArdupilotMode != nullptr)
        delete m_ArdupilotMode;

    m_TrackingManager->clearTrajectoryQueue();
    delete m_TrackingManager; m_TrackingManager = nullptr;
    delete m_TargetController; m_TargetController = nullptr;
}

void VehicleObject_Arduplane::shutdownTargetTracking()
{
    m_TargetController->shutdownTargetController();
    m_TrackingManager->clearTrajectoryQueue();
}

