#ifndef COSTMAP_BASE_LAYER_H
#define COSTMAP_BASE_LAYER_H

#include <string>
#include "common/common.h"
#include "../costmap_2d.h"

namespace mace{
namespace costmap{

class Costmap_BaseLayer : public Costmap2D
{
public:
    Costmap_BaseLayer(const std::string &layerName);

    Costmap_BaseLayer(const std::string &layerName, const uint8_t &fill_value, const double &x_length = 10.0, const double &y_length = 10.0, const double &resolution = 0.5,
                      const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(), const double &rotation = 0.0);

    Costmap_BaseLayer(const std::string &layerName, const uint8_t &fill_value,
               const double &x_min, const double &x_max,
               const double &y_min, const double &y_max,
               const double &resolution,
               const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(),
               const double &rotation = 0.0);

    virtual ~Costmap_BaseLayer() = default;

    void initialize(const std::string &layerName);

    virtual void updateCosts(Costmap2D& master_grid, unsigned int minXIndex, unsigned int minYIndex, unsigned int maxXIndex, unsigned int maxYIndex)
    {
        UNUSED(master_grid); UNUSED(minXIndex); UNUSED(minYIndex); UNUSED(maxXIndex); UNUSED(maxYIndex);
    }

protected:

    void updateWithTrueOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void updateWithOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void updateWithMax(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void updateWithAddition(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

    void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);

    bool has_extra_bounds_;

protected:
    double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

} //end of namespace costmap
} //end of namespace mace

#endif // COSTMAP_BASE_LAYER_H
