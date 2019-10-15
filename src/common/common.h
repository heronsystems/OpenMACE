#ifndef COMMON_H
#define COMMON_H

#define UNUSED(x) (void)(x)

#include <typeinfo>

template< typename Type, typename DataType >
inline bool isOfType( const DataType& data ){
    if( &data == nullptr ) return false;
    return typeid( data ) == typeid( Type );
}

#include "enum_class_hash.h"
#include "publisher.h"

#include <cstddef>

#endif // COMMON_H
