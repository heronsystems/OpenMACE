/*!
  * @file abstract_trajectory_management.cpp
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

#include "abstract_trajectory_management.h"

TrajectoryManagement_Base::TrajectoryManagement_Base(const std::string &agentParamsINIPath)
{
    UNUSED(agentParamsINIPath);
}

TrajectoryManagement_Base::~TrajectoryManagement_Base()
{
}

void TrajectoryManagement_Base::start()
{
    Thread::start();
}

void TrajectoryManagement_Base::run()
{

}

void TrajectoryManagement_Base::updatedAgentState(const VehicleState_Cartesian3D &state)
{
    m_CompleteAgentState.set(state);
}

void TrajectoryManagement_Base::clearTrajectoryQueue()
{
    m_TrajectoryQueue.clearQueue();
}

void TrajectoryManagement_Base::receivedNewTrajectory(const VectorStateQueue &trajectory)
{
    m_TrajectoryQueue.insertReferenceTrajectory(trajectory);
}

std::vector<pose::CartesianPosition_3D> TrajectoryManagement_Base::retrieveCurrentHorizon() const
{
    VectorStateQueue horizonTrajectory;
    m_TrajectoryQueue.retrieveImmediateHorizon(horizonTrajectory);
    
    std::vector<pose::CartesianPosition_3D> rtnHorizon;

    for(size_t i = 0; i < horizonTrajectory.size(); i++)
    {
        pose::CartesianPosition_3D vertex;
        Eigen::Vector3d pos = horizonTrajectory.at(i)._position;
        vertex.updatePosition(pos(0),pos(1),pos(2));
        rtnHorizon.push_back(vertex);
    }

    return rtnHorizon;
}

std::vector<pose::CartesianPosition_3D> TrajectoryManagement_Base::retrieveFullHorizon() const
{
    VectorStateQueue horizonTrajectory;
    m_TrajectoryQueue.retrieveFullHorizon(horizonTrajectory);
    
    std::vector<pose::CartesianPosition_3D> rtnHorizon;

    for(size_t i = 0; i < horizonTrajectory.size(); i++)
    {
        pose::CartesianPosition_3D vertex;
        Eigen::Vector3d pos = horizonTrajectory.at(i)._position;
        vertex.updatePosition(pos(0),pos(1),pos(2));
        rtnHorizon.push_back(vertex);
    }

    return rtnHorizon;
}

void TrajectoryManagement_Base::RemoveHost(void *ptr)
{
    _mutex_TargetUpdate.lock();
    m_CartesianTargetLambda.erase(ptr);
    _mutex_TargetUpdate.unlock();
}

void TrajectoryManagement_Base::setLambda_TargetUpdate(const std::function<void(const pose::CartesianPosition_3D &target)> &lambda)
{
    _mutex_TargetUpdate.lock();
    if (m_CartesianTargetLambda.find(nullptr) != m_CartesianTargetLambda.cend())
        m_CartesianTargetLambda.erase(nullptr);
    m_CartesianTargetLambda.insert({nullptr, lambda});
    _mutex_TargetUpdate.unlock();
}

void TrajectoryManagement_Base::addLambda_TargetUpdate(void *host, const std::function<void(const pose::CartesianPosition_3D &target)> &lambda)
{
    _mutex_TargetUpdate.lock();
    m_CartesianTargetLambda.insert({host, lambda});
    _mutex_TargetUpdate.unlock();
}

void TrajectoryManagement_Base::on_TargetUpdate(const pose::CartesianPosition_3D &target)
{
    _mutex_TargetUpdate.lock();
    for (auto it = m_CartesianTargetLambda.cbegin(); it != m_CartesianTargetLambda.cend(); ++it)
        it->second(target);
    _mutex_TargetUpdate.unlock();
}
