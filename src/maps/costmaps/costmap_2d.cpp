#include "costmap_2d.h"

namespace mace{
namespace costmap{

Costmap2D::Costmap2D(const uint8_t &fill_value, const double &x_length, const double &y_length, const double &resolution,
                     const pose::CartesianPosition_2D &position, const double &rotation):
    maps::Data2DGrid<uint8_t>(&fill_value, x_length, y_length, resolution, resolution, position, rotation)
{

}

Costmap2D::Costmap2D(const Costmap2D &copy):
    maps::Data2DGrid<uint8_t>(copy)
{

}

Costmap2D::~Costmap2D()
{

}
size_t Costmap2D::getEquivalentCellDistance(const double &distance) const
{
    double cells_dist = std::max(0.0, ceil(distance / getCellResolution())); //floating to the ceiling should get it over the next cell
    return static_cast<size_t>(cells_dist);
}

void Costmap2D::setCost(const unsigned int &mapX, const unsigned int &mapY, const uint8_t &cost)
{

}

uint8_t Costmap2D::getCost(const unsigned int &mapX, const unsigned int &mapY)
{

}


} //end of namespace costmap
} //end of namespace mace
