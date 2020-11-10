#include "trajectory_point.h"

TrajectoryPoint::TrajectoryPoint()
{
    _position.setZero();
    _velocity.setZero();
    _acceleration.setZero();

    _jerk.setZero();
    _snap.setZero();

    _orientation.setIdentity();
    _rotationalVelocity.setZero();
    _rotationalAcceleration.setZero();
}

TrajectoryPoint::TrajectoryPoint(const TrajectoryPoint &obj)
{
    _time = obj._time;

    _position = obj._position;
    _velocity = obj._velocity;
    _acceleration = obj._acceleration;
    _jerk = obj._jerk;
    _snap = obj._snap;

    _orientation = obj._orientation;
    _rotationalVelocity = obj._rotationalVelocity;
    _rotationalAcceleration = obj._rotationalAcceleration;
}

TrajectoryPoint::~TrajectoryPoint()
{
}

std::string TrajectoryPoint::printTrajectoryInfo() const
{
    std::string str = "(" + std::to_string(_position(0)) + ", " + std::to_string(_position(1)) + ", " + std::to_string(_position(2)) + ")";
    return str;
}

std::ostream &operator<<(std::ostream &os, const TrajectoryPoint &t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Trajectory Point: " << t.printTrajectoryInfo();
    os << stream.str();

    return os;
}
