#ifndef OBJECT_INT_TUPLE_H
#define OBJECT_INT_TUPLE_H

#include <stdlib.h>
#include <hashtable.h>

//!
//! \brief A Class that Wraps up an object along with an int in a tuple.
//!
template <typename T>
class ObjectIntTuple
{
public:

    T m_obj;
    int m_int;

public:
    ObjectIntTuple(const T &obj, const int &integer) :
        m_obj(obj),
        m_int(integer)
    {

    }

    bool operator== (const ObjectIntTuple &rhs) const
    {
        if(this->m_obj != rhs.m_obj)
            return false;
        if(this->m_int != rhs.m_int) {
            return false;
        }
        return true;
    }

    bool operator != (const ObjectIntTuple &rhs) const
    {
        return !(*this == rhs);
    }
};


namespace std
{

template <typename T>
struct hash<ObjectIntTuple<T>>
{
    std::size_t operator()(const ObjectIntTuple<T>& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;



      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

        std::size_t const h1 ( std::hash<T>{}(k.m_obj) );
        std::size_t const h2 ( std::hash<int>{}((int)k.m_int) );
        return h1 ^ (h2 << 1);
    }
};
}

#endif // OBJECT_INT_TUPLE_H
