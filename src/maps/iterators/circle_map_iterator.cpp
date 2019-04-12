#include "circle_map_iterator.h"

namespace mace{
namespace maps{

CircleMapIterator::CircleMapIterator(const BaseGridMap *map, const pose::CartesianPosition_2D &origin, const double &radius)
{
    it = new GenericMapIterator(map);
    this->origin = origin;
    this->radius = radius;

    boundSubmap(this->origin,this->radius);
    findValidStartIndex();
}

CircleMapIterator::CircleMapIterator(const BaseGridMap *map, const unsigned int &index, const double &radius)
{
    it = new GenericMapIterator(map);
    double posX, posY;
    map->getPositionFromIndex(index,posX,posY);

    this->origin = pose::CartesianPosition_2D(posX,posY);
    this->radius = radius;

    boundSubmap(this->origin,this->radius);
    findValidStartIndex();
}

CircleMapIterator::CircleMapIterator(const CircleMapIterator *copy)
{
    this->it = new GenericMapIterator(copy->it->parentMap);
    this->origin = copy->origin;
    this->radius = copy->radius;
}

CircleMapIterator CircleMapIterator::begin() const
{
    CircleMapIterator newIT(this);
     newIT.it->setCurrentIndex(newIT.it->getStartIndex());
    return newIT;
}

CircleMapIterator CircleMapIterator::end() const
{
    CircleMapIterator newIT(this);
    newIT.it->setCurrentIndex(newIT.it->getEndIndex());
    return newIT;
}

bool CircleMapIterator::isPastEnd() const
{
    return it->isPastEnd();
}

CircleMapIterator& CircleMapIterator::operator =(const CircleMapIterator& rhs)
{
    this->it = new GenericMapIterator(rhs.it->parentMap);
    this->radius = rhs.radius;
    this->origin = rhs.origin;
    return *this;
}

bool CircleMapIterator::operator == (const CircleMapIterator &rhs) const
{
    if(this->it != rhs.it){
        return false;
    }
    if(this->radius != rhs.radius){
        return false;
    }
    if(this->origin != rhs.origin){
        return false;
    }
    return true;
}

bool CircleMapIterator::operator != (const CircleMapIterator &rhs) const
{
    return !(*this == rhs);
}


CircleMapIterator& CircleMapIterator::operator ++()
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

CircleMapIterator CircleMapIterator::operator ++(int)
{
    CircleMapIterator old(*this);
    ++(*this);
    return old;
}

int CircleMapIterator::operator *() const
{
    return *(*it);
}


bool CircleMapIterator::isInside() const
{
    double x,y;
    it->parentMap->getPositionFromIndex(it->getCurrentIndex(),x,y);
    pose::CartesianPosition_2D currentPosition(x,y);
    if(currentPosition.distanceBetween2D(this->origin) <= this->radius)
        return true;
    return false;
}

void CircleMapIterator::boundSubmap(const pose::CartesianPosition_2D &origin, const double &radius)
{
    //First, grab the most furthest edges of the iterator. We will need to map this into the actual map space
    //This will prvent iterators that are on the edges of the map going oob
    pose::CartesianPosition_2D bottomLeft(origin.getXPosition() - radius, origin.getYPosition() - radius);
    pose::CartesianPosition_2D upperRight(origin.getXPosition() + radius, origin.getYPosition() + radius);

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

void CircleMapIterator::findValidStartIndex()
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
