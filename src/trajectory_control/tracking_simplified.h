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

#include "abstract_trajectory_management.h"

//The goal of this function is to always use the current reference before moving on 
class TrackingSimplified : public TrajectoryManagement_Base
{
public:
    TrackingSimplified();

    TrackingSimplified(const std::string &agentParamsINIPath);
    
    TrackingSimplified(const TrackingSimplified &copy) = delete;

    virtual ~TrackingSimplified();

private:
  Data::DataGetSetNotifier<pose::CartesianPosition_3D> m_TargetPosition;
  pose::Rotation_3D m_TargetRotation;
  
  TrajectoryPoint m_TargetPoint;

  void reassessForTrajectoryQueue();
  
  bool checkTrajectoryProgression(const VehicleState_Cartesian3D &currentState);
    
};

#endif // TRACKING_SIMPLIFIED_H
