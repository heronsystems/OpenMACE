#ifndef COSTMAP_BASE_LAYER_H
#define COSTMAP_BASE_LAYER_H

#include <string>
#include "../costmap_2d.h"

namespace mace{
namespace costmap{

class Costmap_BaseLayer : public Costmap2D
{
public:

    Costmap_BaseLayer(const std::string &layerName);

    Costmap_BaseLayer(const uint8_t &fill_value, const double &x_length = 10.0, const double &y_length = 10.0, const double &resolution = 0.5,
                      const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(), const double &rotationAngleDegrees = 0.0);

    virtual ~Costmap_BaseLayer() = default;

    void initialize(const std::string &layerName);

    virtual void updateCosts()
    {

    }

protected:
    /*
           * Updates the master_grid within the specified
           * bounding box using this layer's values.
           *
           * TrueOverwrite means every value from this layer
           * is written into the master grid.
           */
    void updateWithTrueOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    /*
           * Updates the master_grid within the specified
           * bounding box using this layer's values.
           *
           * Overwrite means every valid value from this layer
           * is written into the master grid (does not copy NO_INFORMATION)
           */
    void updateWithOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    /*
           * Updates the master_grid within the specified
           * bounding box using this layer's values.
           *
           * Sets the new value to the maximum of the master_grid's value
           * and this layer's value. If the master value is NO_INFORMATION,
           * it is overwritten. If the layer's value is NO_INFORMATION,
           * the master value does not change.
           */
    void updateWithMax(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    /*
           * Updates the master_grid within the specified
           * bounding box using this layer's values.
           *
           * Sets the new value to the sum of the master grid's value
           * and this layer's value. If the master value is NO_INFORMATION,
           * it is overwritten with the layer's value. If the layer's value
           * is NO_INFORMATION, then the master value does not change.
           *
           * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
           * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
           */
    void updateWithAddition(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    /**
           * Updates the bounding box specified in the parameters to include
           * the location (x,y)
           *
           * @param x x-coordinate to include
           * @param y y-coordinate to include
           * @param min_x bounding box
           * @param min_y bounding box
           * @param max_x bounding box
           * @param max_y bounding box
           */
    void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

    /*
           * Updates the bounding box specified in the parameters
           * to include the bounding box from the addExtraBounds
           * call. If addExtraBounds was not called, the method will do nothing.
           *
           * Should be called at the beginning of the updateBounds method
           *
           * @param min_x bounding box (input and output)
           * @param min_y bounding box (input and output)
           * @param max_x bounding box (input and output)
           * @param max_y bounding box (input and output)
           */
    void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);
    bool has_extra_bounds_;

private:
    double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

} //end of namespace costmap
} //end of namespace mace

#endif // COSTMAP_BASE_LAYER_H
