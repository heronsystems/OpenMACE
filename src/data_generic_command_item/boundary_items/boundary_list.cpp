#include "boundary_list.h"

#include <exception>

namespace BoundaryItem {

BoundaryList::BoundaryList() :
    boundingPolygon(BoundaryTypeToString(BOUNDARYTYPE::GENERIC_POLYGON))
{

}


BoundaryList::BoundaryList(const BoundaryList &rhs)
{
    this->boundingPolygon = rhs.boundingPolygon;
}

void BoundaryList::initializeBoundary(const unsigned int &size)
{
    boundingPolygon.initializePolygon(size);
}

void BoundaryList::clearQueue()
{
    boundingPolygon.clearPolygon();
}

void BoundaryList::appendVertexItem(const Abstract_CartesianPosition* vertexItem)
{
    if(vertexItem->isGreaterThan1D())
        boundingPolygon.appendVertex(*vertexItem->positionAs<CartesianPosition_2D>());
}

void BoundaryList::replaceVertexItemAtIndex(const Abstract_CartesianPosition *vertexItem, const unsigned int &index)
{
    if(vertexItem->isGreaterThan1D())
        boundingPolygon.insertVertexAtIndex(*vertexItem->positionAs<CartesianPosition_2D>(), index);
}

CartesianPosition_2D BoundaryList::getBoundaryItemAtIndex(const unsigned int &index) const
{
    CartesianPosition_2D vertex = boundingPolygon.at(static_cast<int>(index));
    return vertex;
}

size_t BoundaryList::getQueueSize() const
{
    return boundingPolygon.polygonSize();
}

BoundaryList::BoundaryListStatus BoundaryList::getBoundaryListStatus() const
{
    BoundaryListState boundaryState = BoundaryListState::COMPLETE;
    std::vector<int> nullItems = boundingPolygon.findUndefinedVertices();
    if(nullItems.size() > 0)
        boundaryState = BoundaryListState::INCOMPLETE;

    BoundaryListStatus boundaryStatus;
    boundaryStatus.state = boundaryState;
    boundaryStatus.remainingItems = nullItems;

    return boundaryStatus;
}


std::ostream& operator<<(std::ostream& os, const BoundaryList& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed
           <<", Size: " << std::to_string(t.getQueueSize()) << ".";
    os << stream.str();

    return os;
}

}//end of namespace BoundaryItem

