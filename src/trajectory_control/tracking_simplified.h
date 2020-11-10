/*!
  * @file tracking_simplified.h
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
  *     March 2020
  *
  * @copyright
  *     File and its related contents are subjected to a proprietary software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#ifndef TRACKING_SIMPLIFIED_H
#define TRACKING_SIMPLIFIED_H

#include "vehicle/commands/command_item_components.h"

#include "common/msg/command_state.hpp"
namespace common_msg = common::msg;

#include "abstract_trajectory_management.h"



//The goal of this function is to always use the current reference before moving on 
class TrackingSimplified : public TrajectoryManagement_Base
{
public:
    TrackingSimplified(const std::string &agentParamsINIPath, const std::string &environmentINIPath, const std::string &scenarioINIPath);

    TrackingSimplified();
    
    virtual ~TrackingSimplified();

private:
  pose::CartesianPosition_3D m_TargetPosition;
  pose::Rotation_3D m_TargetRotation;
  
  void reassessForTrajectoryQueue();
  
  void checkTrajectoryProgression(const VehicleState_Cartesian3D &currentState);
    
};

#endif // TRACKING_SIMPLIFIED_H
