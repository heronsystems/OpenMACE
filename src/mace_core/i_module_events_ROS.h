#ifndef I_MODULE_EVENTS_ROS_H
#define I_MODULE_EVENTS_ROS_H

#include "i_module_events_general.h"
#include "i_module_events_path_planning.h"

#include "base/pose/pose.h"
#include "base/pose/rotation_3D.h"
#include "base/pose/cartesian_position_3D.h"

namespace MaceCore
{

class IModuleEventsROS  : virtual public IModuleEventsPathPlanning
{

public:
    //!
    //! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
    //! \param obj Point cloud object
    //! \param position Position of sensor
    //!
    virtual void ROS_NewLaserScan(const octomap::Pointcloud& obj, const mace::pose::CartesianPosition_3D &position) = 0;

    //!
    //! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
    //! \param obj Point cloud object
    //! \param position Position of sensor
    //! \param orientation Orientation of sensor
    //!
    virtual void ROS_NewLaserScan(const octomap::Pointcloud& obj, const mace::pose::CartesianPosition_3D &position, const mace::pose::Rotation_3D &orientation) = 0;

    //!
    //! \brief ROS_NewVisionPoseEstimate
    //! \param pose
    //!
    virtual void ROS_NewVisionPoseEstimate(const unsigned int &vehicleID, const mace::pose::Pose &pose) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_ROS_H
