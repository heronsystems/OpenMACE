#ifndef COSTMAP_2D_H
#define COSTMAP_2D_H

#include <cmath>

#include "../data_2d_grid.h"

namespace mace {
namespace costmap {

class Costmap2D : public maps::Data2DGrid<uint8_t>
{
public:
    enum CostValue
    {
        NO_INFORMATION = 255,
        LETHAL_OBSTACLE = 254,
        INSCRIBED_OCCUPANY = 253,
        CIRCUMSCRIBED_OCCUPANCY = 252,
        FREE_SPACE = 0
    };

public:
    Costmap2D(const uint8_t &fill_value = NO_INFORMATION, const double &x_length = 10.0, const double &y_length = 10.0, const double &resolution = 0.5,
              const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(), const double &rotation = 0.0);

    Costmap2D(const uint8_t &fill_value,
               const double &x_min, const double &x_max,
               const double &y_min, const double &y_max,
               const double &resolution,
               const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(),
               const double &rotation = 0.0);

    Costmap2D(const Costmap2D &copy);

    ~Costmap2D();

    /**
     * @brief  Given distance in the world... convert it to cells
     * @param  world_dist The world distance
     * @return The equivalent cell distance
     */
    size_t getEquivalentCellDistance(const double &euclidianDistance) const;

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

    bool isLayerCurrent() const
    {
        return m_LayerCurrent;
    }

    double getCellResolution() const
    {
        return this->xResolution; //can return either since in a costmap we are enforcing the X and Y dimensions to remain the same
    }

protected:
    bool m_LayerEnabled = true;
    bool m_LayerCurrent = false;

public:
    void setScribedRadii(const double inscribed, const double &circumscribed = 0.0);

    double getInscribedRadius() const
    {
        return this->m_InscribedRadius;
    }
    double getCircumscribedRadius() const
    {
        return this->m_CircumscribedRadius;
    }
protected:
    double m_InscribedRadius = 0.0;
    double m_CircumscribedRadius = 0.0;
};

} //end of namespace maps
} //end of namespace mace

#endif // COSTMAP_2D_H
