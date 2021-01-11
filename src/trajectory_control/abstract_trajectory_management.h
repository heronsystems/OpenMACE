/*!
  * @file abstract_trajectory_management.h
  * 
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *     This is a part of Heron Systems participation within the APL's Skyborg Program exercising TACE with existing autonomy. 
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
  
#ifndef ABSTRACT_TRAJECTORY_MANAGEMENT_H
#define ABSTRACT_TRAJECTORY_MANAGEMENT_H

#include <vector>
#include "common/common.h"
#include "data/data_get_set_notifier.h"
#include "common/thread_manager.h"

#include "base/ini_support/INIHelper.h"
#include "base/trajectory/trajectory_point.h"

#include "base/vehicle/vehicle_state.h"

#include "base/trajectory/agent_parameters.h"
#include "base/trajectory/trajectory_queue.h"

using namespace mace;

class TrajectoryManagement_Base : public Thread
{
public:
    TrajectoryManagement_Base(const std::string &agentParamsINIPath = "");

    TrajectoryManagement_Base(const TrajectoryManagement_Base &copy) = delete;

    virtual ~TrajectoryManagement_Base();
    
public: //interface imposed through inheritance of Thread
    void start() override;

    void run() override;

    void updateAgentParams(const AgentParams &params); 

    void updatedAgentState(const VehicleState_Cartesian3D &state);

    void clearTrajectoryQueue();
    
    void receivedNewTrajectory(const VectorStateQueue &trajectory);

    std::vector<pose::CartesianPosition_3D> retrieveCurrentHorizon() const;
    
    std::vector<pose::CartesianPosition_3D> retrieveFullHorizon() const;
    
public:
    virtual void RemoveHost(void *ptr);

    void setLambda_TargetUpdate(const std::function<void(const pose::CartesianPosition_3D &target)> &lambda);

    void addLambda_TargetUpdate(void *host, const std::function<void(const pose::CartesianPosition_3D &target)> &lambda);

protected:
    void on_TargetUpdate(const pose::CartesianPosition_3D &target);

    std::mutex _mutex_TargetUpdate;
    std::unordered_map<void *, std::function<void(const pose::CartesianPosition_3D &target)>> m_CartesianTargetLambda;

public:
    TrajectoryQueue m_TrajectoryQueue;
    
protected:
    AgentParams m_AgentParams;
    Data::DataGetSetNotifier<VehicleState_Cartesian3D> m_CompleteAgentState;

protected:
    unsigned int _timeout = 10;
};

#endif // ABSTRACT_TRAJECTORY_MANAGEMENT_H
