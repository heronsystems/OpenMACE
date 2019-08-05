#include "grid_map_iterator.h"

namespace mace{
namespace maps {


GridMapIterator::GridMapIterator(const BaseGridMap *map)
{
    this->mapSize = map->getNodeCount();
    this->currentIndex = 0;
    this->pastEnd = false;
}

GridMapIterator::GridMapIterator(const GridMapIterator *copy)
{
    this->mapSize = copy->mapSize;
    this->currentIndex = copy->currentIndex;
    this->pastEnd = copy->pastEnd;
}

GridMapIterator GridMapIterator::begin() const
{
    GridMapIterator res(this);
    res.currentIndex = 0;
    return res;
}

GridMapIterator GridMapIterator::end() const
{
    GridMapIterator res(this);
    res.currentIndex = this->mapSize;
    return res;
}

bool GridMapIterator::isPastEnd() const
{
    return pastEnd;
}

GridMapIterator& GridMapIterator::operator =(const GridMapIterator& rhs)
{
    this->mapSize = rhs.mapSize;
    this->currentIndex = rhs.currentIndex;
    this->pastEnd = rhs.pastEnd;
    return *this;
}

bool GridMapIterator::operator == (const GridMapIterator &rhs) const
{
    if(this->mapSize != rhs.mapSize){
        return false;
    }
    if(this->currentIndex != rhs.currentIndex){
        return false;
    }
    if(this->pastEnd != rhs.pastEnd){
        return false;
    }
    return true;
}

bool GridMapIterator::operator != (const GridMapIterator &rhs) const
{
    return !(*this == rhs);
}


GridMapIterator& GridMapIterator::operator ++()
{
    currentIndex++;
    if(currentIndex < this->mapSize)
        pastEnd = false;
    else
        pastEnd = true;

    return *this;
}

GridMapIterator GridMapIterator::operator ++(int)
{
    GridMapIterator old = *this;
    ++*this;
    return old;
}

unsigned int GridMapIterator::operator *() const
{
    return currentIndex;
}

} //end of namepsace mace
} //end of namespace maps

