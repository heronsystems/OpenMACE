#ifndef I_MODULE_EVENTS_ROS_H
#define I_MODULE_EVENTS_ROS_H

#include "i_module_events_general.h"

#include "base/pose/orientation_3D.h"
#include "base/pose/cartesian_position_3D.h"

namespace MaceCore
{

class IModuleEventsROS  : public IModuleEventsGeneral
{

public:
    //!
    //! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
    //! \param obj Point cloud object
    //! \param position Position of sensor
    //!
    virtual void ROS_NewLaserScan(const octomap::Pointcloud& obj, const mace::pose::Position<mace::pose::CartesianPosition_3D>& position) = 0;

    //!
    //! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
    //! \param obj Point cloud object
    //! \param position Position of sensor
    //! \param orientation Orientation of sensor
    //!
    virtual void ROS_NewLaserScan(const octomap::Pointcloud& obj, const mace::pose::Position<mace::pose::CartesianPosition_3D>& position, const mace::pose::Orientation_3D& orientation) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_ROS_H
