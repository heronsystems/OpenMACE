#include "rotation_2D.h"

namespace mace {
namespace pose {

Rotation_2D::Rotation_2D(const std::string &name):
    AbstractOrientation(name), Eigen::Rotation2D<double>()
{
}

Rotation_2D::Rotation_2D(const Rotation_2D &copy):
    AbstractOrientation(copy), Eigen::Rotation2D<double> (copy.angle())
{

}

Rotation_2D::Rotation_2D(const double &angle):
    AbstractOrientation(), Eigen::Rotation2D<double>(angle)
{

}

void Rotation_2D::setPhi(const double &angle)
{
    this->angle() = angle;
}

double Rotation_2D::getPhi() const
{
    return this->angle();
}


} //end of namespace pose
} //end of namespace mace
