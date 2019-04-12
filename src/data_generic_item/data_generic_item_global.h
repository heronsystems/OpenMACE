#ifndef DATA_GENERIC_ITEM_GLOBAL_H
#define DATA_GENERIC_ITEM_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_GENERIC_ITEM_LIBRARY)
#    define DATA_GENERIC_ITEMSHARED_EXPORT __declspec(dllexport)
#  else
#    define DATA_GENERIC_ITEMSHARED_EXPORT __declspec(dllexport)
#  endif
#else
#  define DATA_GENERIC_ITEMSHARED_EXPORT
#endif

#endif // DATA_GENERIC_ITEM_GLOBAL_H
