#ifndef COSTMAP_INFLATION_LAYER_H
#define COSTMAP_INFLATION_LAYER_H

#include <cmath>
#include <limits>
#include <algorithm>

#include "../cell_data.h"

#include "costmap_base_layer.h"

namespace mace{
namespace costmap{

class Costmap_InflationLayer : public Costmap_BaseLayer
{
public:
    Costmap_InflationLayer(const std::string &layerName);

    Costmap_InflationLayer(const std::string &layerName, const uint8_t &fill_value, const double &x_length = 10.0, const double &y_length = 10.0, const double &resolution = 0.5,
                      const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(), const double &rotation = 0.0);

    Costmap_InflationLayer(const std::string &layerName, const uint8_t &fill_value,
               const double &x_min, const double &x_max,
               const double &y_min, const double &y_max,
               const double &resolution,
               const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(),
               const double &rotation = 0.0);


    virtual ~Costmap_InflationLayer() override
    {
        deleteKernels();
        if (seen_)
            delete[] seen_;
    }

    virtual void onInitialize();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);

    void updateCosts(Costmap2D& masterMap, unsigned int minXIndex, unsigned int minYIndex, unsigned int maxXIndex, unsigned int maxYIndex) override;

    virtual bool isDiscretized()
    {
        return true;
    }

    virtual void matchSize();

    virtual void reset() { onInitialize(); }

    /** @brief  Given a distance, compute a cost.
     * @param  distance The distance from an obstacle in cells
     * @return A cost value for the distance */
    virtual inline uint8_t computeCost(const double &distance) const
    {
        double euclidean_distance = distance * getCellResolution();
        uint8_t cost = 0;
        if (std::fabs(distance) <= std::numeric_limits<double>::epsilon())
            cost = LETHAL_OBSTACLE;
        else if (euclidean_distance <= m_InscribedRadius)
            cost = INSCRIBED_OCCUPANY;
        else if (euclidean_distance <= m_CircumscribedRadius)
            cost = CIRCUMSCRIBED_OCCUPANCY;
        else // make sure cost falls off by Euclidean distance
        {
            double factor = exp(-1.0 * weight_ * (euclidean_distance - m_CircumscribedRadius));
            cost = static_cast<uint8_t>((CIRCUMSCRIBED_OCCUPANCY - 1) * factor);
        }
        return cost;
    }

    /**
       * @brief Change the values of the inflation radius parameters
       * @param inflation_radius The new inflation radius
       * @param cost_scaling_factor The new weight
       */
    void setInflationParameters(double inflation_radius, double cost_scaling_factor);

protected:
    virtual void onFootprintChanged();
//    boost::recursive_mutex* inflation_access_;

private:
    /**
       * @brief  Lookup pre-computed distances
       * @param mx The x coordinate of the current cell
       * @param my The y coordinate of the current cell
       * @param src_x The x coordinate of the source cell
       * @param src_y The y coordinate of the source cell
       * @return
       */
    inline double distanceLookup(unsigned int xIndex, unsigned int yIndex, unsigned int srcXIndex, unsigned int srcYIndex)
    {
        unsigned int dx = static_cast<unsigned int>(std::abs<int>(static_cast<int>(xIndex) - static_cast<int>(srcXIndex)));
        unsigned int dy = static_cast<unsigned int>(std::abs<int>(static_cast<int>(yIndex) - static_cast<int>(srcYIndex)));
        return cached_distances_[dx][dy];
    }

    /**
       * @brief  Lookup pre-computed costs
       * @param mx The x coordinate of the current cell
       * @param my The y coordinate of the current cell
       * @param src_x The x coordinate of the source cell
       * @param src_y The y coordinate of the source cell
       * @return
       */
    inline unsigned char costLookup(unsigned int xIndex, unsigned int yIndex, unsigned int srcXIndex, unsigned int srcYIndex)
    {
        unsigned int dx = static_cast<unsigned int>(std::abs<int>(static_cast<int>(xIndex) - static_cast<int>(srcXIndex)));
        unsigned int dy = static_cast<unsigned int>(std::abs<int>(static_cast<int>(yIndex) - static_cast<int>(srcYIndex)));
        return cached_costs_[dx][dy];
    }

    void computeCaches();

    void deleteKernels();

    inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                        unsigned int src_x, unsigned int src_y);

private:
    /*
    inflation_radius_(0)
      , weight_(0)
      , inflate_unknown_(false)
      , cell_inflation_radius_(0)
      , cached_cell_inflation_radius_(0)
    */

    size_t cell_inflation_radius_ = 0;
    size_t cached_cell_inflation_radius_ = 0;
    std::map<double, std::vector<CellData> > inflation_cells_;

    bool* seen_ = nullptr;
    size_t seen_size_;

    double inflation_radius_;
    double weight_;
    bool inflate_unknown_;

    uint8_t** cached_costs_ = nullptr;
    double** cached_distances_ = nullptr;
    double last_min_x_ = -std::numeric_limits<double>::max(), last_min_y_ = -std::numeric_limits<double>::max();
    double last_max_x_ = std::numeric_limits<double>::max(), last_max_y_ = std::numeric_limits<double>::max();

    bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};


} //end of namespace costmap
} //end of namespace mace

#endif // COSTMAP_INFLATION_LAYER_H
