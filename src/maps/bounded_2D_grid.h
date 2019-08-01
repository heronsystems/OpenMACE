#ifndef BOUNDED_2D_GRID_H
#define BOUNDED_2D_GRID_H

#include <unordered_map>

#include "base/pose/abstract_position.h"
#include "base/pose/cartesian_position_2D.h"

#include "base/geometry/polygon_cartesian.h"

#include "dynamic_2D_grid.h"


namespace mace {
namespace maps {

using namespace pose;

class Bounded2DGrid : public Dynamic2DGrid<Position<CartesianPosition_2D>>
{
public:
    Bounded2DGrid(const double &x_min = -1.0, const double &x_max = 1.0,
                  const double &y_min = -1.0, const double &y_max = 1.0,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                   const Position<CartesianPosition_2D> *fill_value = nullptr);

    Bounded2DGrid(const geometry::Polygon_Cartesian &boundingPolygon,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                  const Position<CartesianPosition_2D> *fill_value = nullptr);

    std::vector<Position<CartesianPosition_2D> *> setBoundingPolygon(const geometry::Polygon_Cartesian &polygon);

    std::vector<Position<CartesianPosition_2D> *> getBoundedDataVector() const;

    std::list<Position<CartesianPosition_2D> *> getBoundedDataList() const;

    void print() const;
private:
    void clearData();

protected:
    geometry::Polygon_Cartesian m_boundary;
    std::vector<Position<CartesianPosition_2D> *> m_constrainedData;
};

} //end of namespace maps
} //end of namespace mace

#endif // BOUNDED_2D_GRID_H
