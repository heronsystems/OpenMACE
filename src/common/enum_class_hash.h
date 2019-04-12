#ifndef ENUM_CLASS_HASH_H
#define ENUM_CLASS_HASH_H

#include <cstdlib>

struct EnumClassHash
{
    template <typename ET>
    std::size_t operator()(ET t) const
    {
        return static_cast<std::size_t>(t);
    }
};

#endif // ENUM_CLASS_HASH_H
