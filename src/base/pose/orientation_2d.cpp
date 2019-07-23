#include "orientation_2D.h"

namespace mace {
namespace pose {

Orientation_2D::Orientation_2D(const std::string &name):
    Eigen::Rotation2D<double>()
{
    this->name = name;
}

Orientation_2D::Orientation_2D(const Orientation_2D &copy):
    Eigen::Rotation2D<double> (copy.angle())
{
    this->name = copy.name;
}

Orientation_2D::Orientation_2D(const double &angle):
    Eigen::Rotation2D<double>(angle)
{

}

void Orientation_2D::setPhi(const double &angle)
{
    this->angle() = angle;
}

double Orientation_2D::getPhi() const
{
    return this->angle();
}


} //end of namespace pose
} //end of namespace mace
