#ifndef VORONOI_2DCELL_H
#define VORONOI_2DCELL_H

#include "polygon_2DC.h"

namespace mace {
namespace geometry {

class Cell_2DC : public Polygon_2DC
{
public:
    Cell_2DC(const std::string &descriptor = "2D Cartesian Polygon");

    Cell_2DC(const std::vector<Position<CartesianPosition_2D>> &vector, const std::string &descriptor);

    Cell_2DC(const Cell_2DC &copy);

    ~Cell_2DC() = default;

    void insertNodes(std::list<Position<CartesianPosition_2D>*> &checkVector, const bool &onLineCheck = false);

    std::vector<Position<CartesianPosition_2D>*> getNodes() const;

private:
    std::vector<Position<CartesianPosition_2D>*> m_nodes;
};

} //end of namespace geometry
} //end of namespace mace

#endif // VORONOI_2DCELL_H
