#ifndef GRID_MAP_ITERATOR_H
#define GRID_MAP_ITERATOR_H

#include <stddef.h>

#include "maps/base_grid_map.h"

namespace mace{
namespace maps {

class GridMapIterator
{
public:
    GridMapIterator(const BaseGridMap *map);

    GridMapIterator(const GridMapIterator* copy);

    virtual ~GridMapIterator() = default;

public:
    GridMapIterator begin() const;

    GridMapIterator end() const;

    bool isPastEnd() const;

public:
    virtual GridMapIterator& operator ++();
    virtual GridMapIterator operator ++(int);

    GridMapIterator& operator =(const GridMapIterator &rhs);

    bool operator == (const GridMapIterator &rhs) const;

    bool operator !=(const GridMapIterator &rhs) const;

    unsigned int operator *() const;

private:
    size_t currentIndex;
    size_t mapSize;

    bool pastEnd;
};

} //end of namepsace mace
} //end of namespace maps


#endif // GRID_MAP_ITERATOR_H
