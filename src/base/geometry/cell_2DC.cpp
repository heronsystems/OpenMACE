#include "cell_2DC.h"

namespace mace {
namespace geometry{

Cell_2DC::Cell_2DC(const std::string &descriptor):
    Polygon_2DC(descriptor)
{

}

Cell_2DC::Cell_2DC(const std::vector<Position<CartesianPosition_2D>> &vector, const std::string &descriptor):
    Polygon_2DC(vector, descriptor)
{

}

Cell_2DC::Cell_2DC(const Cell_2DC &copy):
    Polygon_2DC(copy)
{
    this->m_nodes = copy.m_nodes;
}

void Cell_2DC::insertNodes(std::list<Position<CartesianPosition_2D>*> &checkVector, const bool &onLineCheck)
{
    std::list<Position<CartesianPosition_2D>*>::iterator i = checkVector.begin();
    while (i != checkVector.end())
    {
        if(contains((*i)->getXPosition(),(*i)->getYPosition(),onLineCheck))
        {
            m_nodes.push_back(*i);
            checkVector.erase(i++);
        }
        else
            ++i;
    }
}

std::vector<Position<CartesianPosition_2D>*> Cell_2DC::getNodes() const
{
    return this->m_nodes;
}


} //end of namespace geometry
} //end of namespace mace
