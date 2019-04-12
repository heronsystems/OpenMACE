#ifndef CLASS_FORWARD_H
#define CLASS_FORWARD_H

#include <memory>

/** \brief Macro that defines a forward declaration for a class, and
    shared pointers to the class. */
#define MACE_CLASS_FORWARD(C)                                                                                          \
    class C;                                                                                                           \
    typedef std::shared_ptr<C> C##Ptr

#endif // CLASS_FORWARD_H
