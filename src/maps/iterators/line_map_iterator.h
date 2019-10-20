#ifndef LINEMAPITERATOR_H
#define LINEMAPITERATOR_H

#include "base/pose/cartesian_position_2D.h"

#include "maps/iterators/generic_map_iterator.h"

namespace mace{
namespace maps{

class LineMapIterator
{
public:
    LineMapIterator(const maps::BaseGridMap *map, const pose::CartesianPosition_2D &start, const pose::CartesianPosition_2D &end);

    LineMapIterator(const LineMapIterator* copy);

    virtual ~LineMapIterator()
    {
        if(it)
        {
            delete it;
            it = nullptr;
        }
    }

public:
    LineMapIterator begin() const;

    LineMapIterator end() const;

    bool isPastEnd() const;

    void initializeIterationParameters();

public:
    virtual LineMapIterator& operator ++();

    virtual LineMapIterator operator ++(int);

    LineMapIterator& operator =(const LineMapIterator &rhs);

    bool operator == (const LineMapIterator &rhs) const;

    bool operator !=(const LineMapIterator &rhs) const;

    int operator *() const;

private:
    void findValidStartIndex();

private:
    unsigned int m_StartIndex = 0, m_EndIndex = 0;
    Eigen::Vector2i m_Increment1, m_Increment2;
    int m_Numerator = 0, m_Denominator = 0, m_IterateNumerator = 0;
    unsigned int m_NumCells = 0, m_CurrentCellCount = 0;

    maps::GenericMapIterator* it;
};

} //end of namespace maps
} //end of namespace mace


#endif // LINEMAPITERATOR_H
