#ifndef TRAJECTORY_POINT_H
#define TRAJECTORY_POINT_H

#include <Eigen/Eigen>

#include <sys/time.h>
#include <deque>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "common/common.h"
#include "common/class_forward.h"

#include "data/environment_time.h"

MACE_CLASS_FORWARD(TrajectoryPoint);

typedef std::deque<TrajectoryPoint> VectorStateQueue;

class TrajectoryPoint
{
public:
    typedef std::vector<TrajectoryPoint, Eigen::aligned_allocator<TrajectoryPoint>> Vector;

public:
    TrajectoryPoint();

    TrajectoryPoint(const TrajectoryPoint &obj);

    ~TrajectoryPoint();

public:
    void setFromYaw(double yaw)
    {
        _orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

public:
    TrajectoryPoint& operator=(const TrajectoryPoint &rhs)
    {
        _position = rhs._position;
        _velocity = rhs._velocity;
        _acceleration = rhs._acceleration;
        _jerk = rhs._jerk;
        _snap = rhs._snap;

        _orientation = rhs._orientation;
        _rotationalVelocity = rhs._rotationalVelocity;
        _rotationalAcceleration = rhs._rotationalAcceleration;
        return *this;
    }

    bool operator==(const TrajectoryPoint &rhs)
    {
        if (_position != rhs._position)
            return false;
        if (_velocity != rhs._velocity)
            return false;
        if (_acceleration != rhs._acceleration)
            return false;
        if (_jerk != rhs._jerk)
            return false;
        if (_snap != rhs._snap)
            return false;

        if(!this->_orientation.isApprox(rhs._orientation,std::numeric_limits<double>::epsilon()))
            return false;
        if (_rotationalVelocity != rhs._rotationalVelocity)
            return false;
        if (_rotationalAcceleration != rhs._rotationalAcceleration)
            return false;

        return true;
    }

    bool operator!=(const TrajectoryPoint &rhs)
    {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printTrajectoryInfo
    //! \return
    //!
    std::string printTrajectoryInfo() const;

    friend std::ostream &operator<<(std::ostream &os, const TrajectoryPoint &t);

public:
    Data::EnvironmentTime _time;
    
    Eigen::Vector3d _position;
    Eigen::Vector3d _velocity;
    Eigen::Vector3d _acceleration;
    Eigen::Vector3d _jerk;
    Eigen::Vector3d _snap;

    Eigen::Quaterniond _orientation;
    Eigen::Vector3d _rotationalVelocity;
    Eigen::Vector3d _rotationalAcceleration;
};

#endif // TRAJECTORY_POINT_H
