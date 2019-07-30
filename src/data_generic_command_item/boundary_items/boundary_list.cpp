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

void BoundaryList::initializeBoundary(const int &size)
{
    boundingPolygon.initializePolygon(size);
}

void BoundaryList::clearQueue()
{
    boundingPolygon.clearPolygon();
}

void BoundaryList::appendVertexItem(const Abstract_CartesianPosition* vertexItem)
{
    boundingPolygon.appendVertex(vertexItem);
}

void BoundaryList::replaceVertexItemAtIndex(const Abstract_CartesianPosition *vertexItem, const int &index)
{
    boundingPolygon.insertVertexAtIndex(vertexItem, index);
}

Position<CartesianPosition_2D> BoundaryList::getBoundaryItemAtIndex(const int &index) const
{
    Position<CartesianPosition_2D> vertex = boundingPolygon.at(index);
    return vertex;
}

int BoundaryList::getQueueSize() const
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

