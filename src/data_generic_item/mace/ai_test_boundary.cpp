#include "ai_test_boundary.h"

namespace DataGenericItem {

AI_TestBoundary::AI_TestBoundary()
{
}

AI_TestBoundary::AI_TestBoundary(const AI_TestBoundary &copy)
{
    this->_boundary = copy._boundary;
    this->_ceiling = copy._ceiling;
    this->_floor = copy._floor;
}


void AI_TestBoundary::setBoundary(const mace::geometry::Polygon_2DG &boundary)
{
    this->_boundary = boundary;
}

mace::geometry::Polygon_2DG AI_TestBoundary::getBoundary()
{
    return this->_boundary;
}

} // end of namespace DataGenericItem
