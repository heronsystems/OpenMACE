#ifndef GENERIC_MAP_ITERATOR_H
#define GENERIC_MAP_ITERATOR_H

#include "common/class_forward.h"
#include "maps/base_grid_map.h"

namespace mace{
namespace maps{

MACE_CLASS_FORWARD(GenericMapIterator);

class GenericMapIterator
{
public:
    GenericMapIterator(const BaseGridMap *map);

    GenericMapIterator(const GenericMapIterator &copy);

    virtual ~GenericMapIterator() = default;

public:
    GenericMapIterator begin() const;

    GenericMapIterator end() const;

    bool isPastEnd() const;

public:
    virtual GenericMapIterator& operator ++();
    virtual GenericMapIterator operator ++(int);

    GenericMapIterator& operator =(const GenericMapIterator &rhs);

    bool operator == (const GenericMapIterator &rhs) const;

    bool operator !=(const GenericMapIterator &rhs) const;

    int operator *() const;

public:
    const BaseGridMap *parentMap;

    bool pastEnd;

public:

    size_t getCurrentIndex();

    size_t getStartIndex();

    size_t getEndIndex();

    bool setCurrentIndex(size_t currentIndex);

    void setStartIndex(size_t startIndex);

    void setEndIndex(size_t endIndex);

private:
    size_t currentIndex;
    size_t startIndex;
    size_t endIndex;

};

} //end of namepsace mace
} //end of namespace maps

#endif // GENERIC_MAP_ITERATOR_H
