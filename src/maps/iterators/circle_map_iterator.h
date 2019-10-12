#ifndef CIRCLE_MAP_ITERATOR_H
#define CIRCLE_MAP_ITERATOR_H

#include "maps/iterators/generic_map_iterator.h"
#include "base/pose/cartesian_position_2D.h"

#include "maps/base_grid_map.h"

namespace mace{
namespace maps{

class CircleMapIterator
{
public:
    CircleMapIterator(const BaseGridMap *map, const pose::CartesianPosition_2D &origin, const double &radius);

    CircleMapIterator(const BaseGridMap *map, const unsigned int &index, const double &radius);

    CircleMapIterator(const CircleMapIterator* copy);

    virtual ~CircleMapIterator()
    {
        if(it)
        {
            delete it;
            it = nullptr;
        }
    }

public:
    CircleMapIterator begin() const;

    CircleMapIterator end() const;

    bool isPastEnd() const;

public:
    virtual CircleMapIterator& operator ++();

    virtual CircleMapIterator operator ++(int);

    CircleMapIterator& operator =(const CircleMapIterator &rhs);

    bool operator == (const CircleMapIterator &rhs) const;

    bool operator !=(const CircleMapIterator &rhs) const;

    int operator *() const;

private:
    bool isInside() const;

    void boundSubmap(const pose::CartesianPosition_2D &origin, const double &radius);

    void findValidStartIndex();

private:
    GenericMapIterator* it;

    double radius;
    pose::CartesianPosition_2D origin;
};

} //end of namepsace mace
} //end of namespace maps

#endif // CIRCLE_MAP_ITERATOR_H
