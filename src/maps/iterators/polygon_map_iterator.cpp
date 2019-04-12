#include "polygon_map_iterator.h"


namespace mace{
namespace maps{

PolygonMapIterator::PolygonMapIterator(const BaseGridMap *map, const geometry::Polygon_2DC &polygon)
{
    it = new GenericMapIterator(map);
    this->polygon = polygon;

    boundSubmap(this->polygon);
    findValidStartIndex();
}

PolygonMapIterator::PolygonMapIterator(const PolygonMapIterator *copy)
{
    this->it = new GenericMapIterator(copy->it->parentMap);
    this->polygon = copy->polygon;
}

PolygonMapIterator PolygonMapIterator::begin() const
{
    PolygonMapIterator newIT(this);
     newIT.it->setCurrentIndex(newIT.it->getStartIndex());
    return newIT;
}

PolygonMapIterator PolygonMapIterator::end() const
{
    PolygonMapIterator newIT(this);
    newIT.it->setCurrentIndex(newIT.it->getEndIndex());
    return newIT;
}

bool PolygonMapIterator::isPastEnd() const
{
    return it->isPastEnd();
}

PolygonMapIterator& PolygonMapIterator::operator =(const PolygonMapIterator& rhs)
{
    this->it = new GenericMapIterator(rhs.it->parentMap);
    this->polygon = rhs.polygon;
    return *this;
}

bool PolygonMapIterator::operator == (const PolygonMapIterator &rhs) const
{
    if(this->it != rhs.it){
        return false;
    }
    if(this->polygon != rhs.polygon){
        return false;
    }
    return true;
}

bool PolygonMapIterator::operator != (const PolygonMapIterator &rhs) const
{
    return !(*this == rhs);
}


PolygonMapIterator& PolygonMapIterator::operator ++()
{
    ++(*it);

    if(!it->isPastEnd()){
        for( ; !it->isPastEnd(); ++(*it))
        {
            if(isInside())
                break;
        }
    }

    return *this;
}

PolygonMapIterator PolygonMapIterator::operator ++(int)
{
    PolygonMapIterator old(*this);
    ++(*this);
    return old;
}

int PolygonMapIterator::operator *() const
{
    return *(*it);
}


bool PolygonMapIterator::isInside() const
{
    double x,y;
    it->parentMap->getPositionFromIndex(it->getCurrentIndex(),x,y);
    mace::pose::Position<mace::pose::CartesianPosition_2D> iteratorPosition("Iterator Position",x,y);
    if(this->polygon.contains(iteratorPosition,true))
        return true;
    return false;
}

void PolygonMapIterator::boundSubmap(const geometry::Polygon_2DC &boundary)
{
    //First, grab the most furthest edges of the iterator. We will need to map this into the actual map space
    //This will prvent iterators that are on the edges of the map going oob
    pose::CartesianPosition_2D bottomLeft = boundary.getBottomLeft();
    pose::CartesianPosition_2D upperRight = boundary.getTopRight();

    //Here we need to take those positions and map them to the actual map space
    //Check X positions
    if(bottomLeft.getXPosition() < this->it->parentMap->getXMin())
        bottomLeft.setXPosition(this->it->parentMap->getXMin());
    if(upperRight.getXPosition() > this->it->parentMap->getXMax())
        upperRight.setXPosition(this->it->parentMap->getXMax());

    //Check Y positions
    if(bottomLeft.getYPosition() < this->it->parentMap->getYMin())
        bottomLeft.setYPosition(this->it->parentMap->getYMin());
    if(upperRight.getYPosition() > this->it->parentMap->getYMax())
        upperRight.setYPosition(this->it->parentMap->getYMax());

    this->it->setStartIndex(this->it->parentMap->indexFromPos(bottomLeft.getXPosition(), bottomLeft.getYPosition()));
    this->it->setEndIndex(this->it->parentMap->indexFromPos(upperRight.getXPosition(), upperRight.getYPosition()));
}

void PolygonMapIterator::findValidStartIndex()
{
    this->it->setCurrentIndex(this->it->getStartIndex());

    if(isInside())
        return;

    for(;!this->isPastEnd();this->operator ++())
    {
        if(this->isInside())
            break;
    }
}

} //end of namespace maps
} //end of namespace mace
