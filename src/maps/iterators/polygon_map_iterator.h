#ifndef POLYGON_MAP_ITERATOR_H
#define POLYGON_MAP_ITERATOR_H

#include "maps/iterators/generic_map_iterator.h"

#include "base/geometry/polygon_2DC.h"
#include "maps/base_grid_map.h"

namespace mace{
namespace maps{

class PolygonMapIterator
{
public:
    PolygonMapIterator(const BaseGridMap *map, const geometry::Polygon_2DC &polygon);

    PolygonMapIterator(const PolygonMapIterator* copy);

    ~PolygonMapIterator()
    {
        if(it)
        {
            delete it;
            it = nullptr;
        }
    }

public:
    PolygonMapIterator begin() const;

    PolygonMapIterator end() const;

    bool isPastEnd() const;

public:
    virtual PolygonMapIterator& operator ++();

    virtual PolygonMapIterator operator ++(int);

    PolygonMapIterator& operator =(const PolygonMapIterator &rhs);

    bool operator == (const PolygonMapIterator &rhs) const;

    bool operator !=(const PolygonMapIterator &rhs) const;

    int operator *() const;

private:
    bool isInside() const;

    void boundSubmap(const geometry::Polygon_2DC &boundary);

    void findValidStartIndex();

private:
    GenericMapIterator* it;
    geometry::Polygon_2DC polygon;
};

} //end of namepsace mace
} //end of namespace maps

#endif // POLYGON_MAP_ITERATOR_H
