#ifndef VORONOI_2DCELL_H
#define VORONOI_2DCELL_H

#include "polygon_cartesian.h"

namespace mace {
namespace geometry {

class Cell_2DC : public Polygon_Cartesian
{
public:
    Cell_2DC(const std::string &descriptor = "2D Cartesian Polygon");

    Cell_2DC(const std::vector<CartesianPosition_2D> &vector, const std::string &descriptor);

    Cell_2DC(const Cell_2DC &copy);

    virtual ~Cell_2DC() = default;

    void insertNodes(std::list<CartesianPosition_2D*> &checkVector, const bool &onLineCheck = false);

    std::vector<CartesianPosition_2D*> getNodes() const;

private:
    std::vector<CartesianPosition_2D*> m_nodes;
};

} //end of namespace geometry
} //end of namespace mace

#endif // VORONOI_2DCELL_H
