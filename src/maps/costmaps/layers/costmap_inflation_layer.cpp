#include "costmap_inflation_layer.h"

namespace mace{
namespace costmap{


Costmap_InflationLayer::Costmap_InflationLayer(const std::string &layerName):
    Costmap_BaseLayer(layerName)
{

}
Costmap_InflationLayer::Costmap_InflationLayer(const std::string &layerName, const uint8_t &fill_value, const double &x_length, const double &y_length, const double &resolution,
                                               const pose::CartesianPosition_2D &position, const double &rotation):
    Costmap_BaseLayer(layerName, fill_value, x_length, y_length, resolution, position, rotation)
{

}


Costmap_InflationLayer::Costmap_InflationLayer(const std::string &layerName, const uint8_t &fill_value,
           const double &x_min, const double &x_max,
           const double &y_min, const double &y_max,
           const double &resolution,
           const pose::CartesianPosition_2D &position,
           const double &rotation):
    Costmap_BaseLayer(layerName, fill_value, x_min, x_max, y_min, y_max, resolution, position, rotation)
{

}

void Costmap_InflationLayer::onInitialize()
{
    m_LayerCurrent = true;

    if (seen_)
        delete[] seen_;
    seen_ = nullptr;
    seen_size_ = 0;

    need_reinflation_ = false;

    matchSize();
}

void Costmap_InflationLayer::matchSize()
{
    /*
    //  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    resolution_ = costmap->getResolution();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();

    unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
    if (seen_)
        delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
    */
}

void Costmap_InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
    if (need_reinflation_)
    {
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        // For some reason when I make these -<double>::max() it does not
        // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
        // -<float>::max() instead.
        *min_x = -std::numeric_limits<double>::max();
        *min_y = -std::numeric_limits<double>::max();
        *max_x = std::numeric_limits<double>::max();
        *max_y = std::numeric_limits<double>::max();
        need_reinflation_ = false;
    }
    else
    {
        double tmp_min_x = last_min_x_;
        double tmp_min_y = last_min_y_;
        double tmp_max_x = last_max_x_;
        double tmp_max_y = last_max_y_;
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
        *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
        *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
        *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
    }
}

void Costmap_InflationLayer::onFootprintChanged()
{
    cell_inflation_radius_ = getEquivalentCellDistance(m_InscribedRadius);
    computeCaches();
    need_reinflation_ = true;
}

void Costmap_InflationLayer::updateCosts(Costmap2D& masterMap, unsigned int minXIndex, unsigned int minYIndex, unsigned int maxXIndex, unsigned int maxYIndex)
{
    //  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    if (!isLayerEnabled() || (cell_inflation_radius_ == 0))
        return;
    int minMapX = static_cast<int>(minXIndex), minMapY = static_cast<int>(minYIndex);
    int maxMapX = static_cast<int>(maxXIndex), maxMapY = static_cast<int>(maxYIndex);

    size_t size_x = masterMap.getSizeX(), size_y = masterMap.getSizeY();

    if (seen_ == nullptr) {
        //    ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }
    else if (seen_size_ != size_x * size_y)
    {
        //    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
        delete[] seen_;
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }
    memset(seen_, false, size_x * size_y * sizeof(bool));

    // We need to include in the inflation cells outside the bounding
    // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
    // up to that distance outside the box can still influence the costs
    // stored in cells inside the box.
    minMapX -= cell_inflation_radius_;
    minMapY -= cell_inflation_radius_;
    maxMapX += cell_inflation_radius_;
    maxMapY += cell_inflation_radius_;

    minMapX = std::max<int>(0, minMapX);
    minMapY = std::max<int>(0, minMapY);
    maxMapX = std::min<int>(static_cast<int>(size_x), maxMapX);
    maxMapY = std::min<int>(static_cast<int>(size_y), maxMapY);

    // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
    // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

    // Start with lethal obstacles: by definition distance is 0.0
    std::vector<CellData>& obs_bin = inflation_cells_[0.0];
    for (size_t j = static_cast<unsigned int>(minMapY); j < static_cast<unsigned int>(maxMapY); j++)
    {
        for (size_t i = static_cast<unsigned int>(minMapX); i < static_cast<unsigned int>(maxMapX); i++)
        {
            size_t currentIndex = 0;
            if(!masterMap.getVectorIndex(currentIndex,static_cast<unsigned int>(i),static_cast<unsigned int>(j)))
                continue;
            uint8_t cost = *masterMap[static_cast<unsigned int>(currentIndex)];
            if (cost == LETHAL_OBSTACLE)
                obs_bin.push_back(CellData(0.0,static_cast<unsigned int>(currentIndex),
                                           static_cast<unsigned int>(i), static_cast<unsigned int>(j),
                                           static_cast<unsigned int>(i), static_cast<unsigned int>(j)));
        }
    }

    // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
    // can overtake previously inserted but farther away cells
    std::map<double, std::vector<CellData> >::iterator bin;
    for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
    {
        for (size_t i = 0; i < bin->second.size(); ++i)
        {
            // process all cells at distance dist_bin.first
            const CellData& cell = bin->second[i];
            unsigned int index = cell.m_VectorIndex;

            // ignore if already visited
            if (seen_[index])
                continue;

            seen_[index] = true;

            unsigned int mx = cell.m_XIndex;
            unsigned int my = cell.m_YIndex;
            unsigned int sx = cell.m_ObstacleXIndex;
            unsigned int sy = cell.m_ObstacleYIndex;

            // assign the cost associated with the distance from an obstacle to the cell
            uint8_t cost = costLookup(mx, my, sx, sy);
            uint8_t* old_cost = masterMap[index];

            if (*old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_OCCUPANY)))
                *old_cost = cost;
            else
                *old_cost = std::max(*old_cost, cost);

            // attempt to put the neighbors of the current cell onto the inflation list
            if (mx > 0)
                enqueue(index - 1, mx - 1, my, sx, sy);
            if (my > 0)
                enqueue(index - static_cast<unsigned int>(size_x), mx, my - 1, sx, sy);
            if (mx < size_x - 1)
                enqueue(index + 1, mx + 1, my, sx, sy);
            if (my < size_y - 1)
                enqueue(index + static_cast<unsigned int>(size_x), mx, my + 1, sx, sy);
        }
    }

    inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
inline void Costmap_InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                            unsigned int src_x, unsigned int src_y)
{
    if (!seen_[index])
    {
        // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
        double distance = distanceLookup(mx, my, src_x, src_y);

        // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
        if (distance > cell_inflation_radius_)
            return;

        // push the cell data onto the inflation list and mark
        inflation_cells_[distance].push_back(CellData(distance, index, mx, my, src_x, src_y));
    }
}

void Costmap_InflationLayer::computeCaches()
{
    if (cell_inflation_radius_ == 0)
        return;

    // based on the inflation radius... compute distance and cost caches
    if (cell_inflation_radius_ != cached_cell_inflation_radius_)
    {
        deleteKernels();

        cached_costs_ = new uint8_t*[cell_inflation_radius_ + 2];
        cached_distances_ = new double*[cell_inflation_radius_ + 2];

        for (size_t i = 0; i <= cell_inflation_radius_ + 1; ++i)
        {
            cached_costs_[i] = new uint8_t[cell_inflation_radius_ + 2];
            cached_distances_[i] = new double[cell_inflation_radius_ + 2];
            for (size_t j = 0; j <= cell_inflation_radius_ + 1; ++j)
            {
                cached_distances_[i][j] = hypot(i, j);
            }
        }

        cached_cell_inflation_radius_ = cell_inflation_radius_;
    }

    for (size_t i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
        for (size_t j = 0; j <= cell_inflation_radius_ + 1; ++j)
        {
            cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
        }
    }
}

void Costmap_InflationLayer::deleteKernels()
{
    if (cached_distances_ != nullptr)
    {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
        {
            if (cached_distances_[i])
                delete[] cached_distances_[i];
        }
        if (cached_distances_)
            delete[] cached_distances_;
        cached_distances_ = nullptr;
    }

    if (cached_costs_ != nullptr)
    {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
        {
            if (cached_costs_[i])
                delete[] cached_costs_[i];
        }
        delete[] cached_costs_;
        cached_costs_ = nullptr;
    }
}

void Costmap_InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
    if (weight_ != cost_scaling_factor ||
            fabs(inflation_radius_ - inflation_radius) > std::numeric_limits<double>::epsilon())
    {
        inflation_radius_ = inflation_radius;
        // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
        // when accessing the cached arrays
//        boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
        cell_inflation_radius_ = getEquivalentCellDistance(inflation_radius_);
        weight_ = cost_scaling_factor;
        need_reinflation_ = true;
        computeCaches();
    }
}

}  // namespace
}
