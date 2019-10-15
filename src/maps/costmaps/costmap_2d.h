#ifndef COSTMAP_2D_H
#define COSTMAP_2D_H

#include "../data_2d_grid.h"

namespace mace {
namespace costmap {

class Costmap2D : public maps::Data2DGrid<uint8_t>
{
public:
    enum CostValue
    {
        NO_INFORMATION = 255,
        TRUE_OCCUPANCY = 254,
        INSCRIBED_OCCUPANY = 253,
        FREE_SPACE = 0
    };

public:
    Costmap2D(const uint8_t &fill_value, const double &x_length = 10.0, const double &y_length = 10.0, const double &resolution = 0.5,
              const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(), const double &rotationAngleDegrees = 0.0);

    Costmap2D(const Costmap2D &copy);

    ~Costmap2D();

    void setCost(const unsigned int &mapX, const unsigned int &mapY, const uint8_t &cost);

    uint8_t getCost(const unsigned int &mapX, const unsigned int &mapY);

    void enableLayer(const bool &enable)
    {
        this->m_LayerEnabled = enable;
    }

    bool isLayerEnabled() const
    {
        return m_LayerEnabled;
    }

private:
    bool m_LayerEnabled = true;
};

} //end of namespace maps
} //end of namespace mace

#endif // COSTMAP_2D_H
