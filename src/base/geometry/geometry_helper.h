#ifndef GEOMETRY_HELPER_H
#define GEOMETRY_HELPER_H

#include <cmath>
#include <limits>

#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace geometry{

inline double isLeftOfInf(const pose::CartesianPosition_2D &p0,
                          const pose::CartesianPosition_2D &p1,
                          const double &x, const double &y)
{

    return ((p1.getXPosition() - p0.getXPosition()) * (y - p0.getYPosition()) - (x - p0.getXPosition()) * (p1.getYPosition() - p0.getYPosition()));
}


inline double isLeftOfInf(const pose::CartesianPosition_2D &p0,
                          const pose::CartesianPosition_2D &p1,
                          const pose::CartesianPosition_2D &p2)
{
    return isLeftOfInf(p0,p1,p2.getXPosition(), p2.getYPosition());
}


inline double distanceToLine(const pose::CartesianPosition_2D &l0, const pose::CartesianPosition_2D &l1, const double &x, const double &y)
{
    double A = x - l0.getXPosition();
    double B = y - l0.getYPosition();
    double C = l1.getXPosition() - l0.getXPosition();
    double D = l1.getYPosition() - l0.getYPosition();

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;

    if (len_sq != 0) //in case of 0 length line
        param = dot / len_sq;

    double dx , dy;

    if (param < 0) {
        dx = x - l0.getXPosition();
        dy = y - l0.getYPosition();
    }
    else if (param > 1) {
        dx = x - l1.getXPosition();
        dy = y - l1.getYPosition();
    }
    else {
        dx = x - (l0.getXPosition() + param * C);
        dy = y - (l0.getYPosition() + param * D);
    }

    return sqrt(dx * dx + dy * dy);
}

inline double distanceToLine(const pose::CartesianPosition_2D &l0, const pose::CartesianPosition_2D &l1, const pose::CartesianPosition_2D &p0)
{
    return distanceToLine(l0,l1,p0.getXPosition(),p0.getYPosition());
}

inline double isOnLine(const pose::CartesianPosition_2D &l0, const pose::CartesianPosition_2D &l1, const double &x, const double &y, const double limit = std::numeric_limits<double>::epsilon())
{
    if(distanceToLine(l0,l1,x,y) < limit)
        return true;
    return false;
}

inline double isOnLine(const pose::CartesianPosition_2D &l0, const pose::CartesianPosition_2D &l1, const pose::CartesianPosition_2D &p0, const double limit = std::numeric_limits<double>::epsilon())
{
    if(distanceToLine(l0,l1,p0.getXPosition(),p0.getYPosition()) < limit)
        return true;
    return false;
}

}
}

#endif // GEOMETRY_HELPER_H
