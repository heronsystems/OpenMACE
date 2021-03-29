#ifndef DATA_INTERFACE_MACE_GLOBAL_H
#define DATA_INTERFACE_MACE_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_INTERFACE_MACE_LIBRARY)
#    define DATA_INTERFACE_MACESHARED_EXPORT __declspec(dllexport)
#  else
#    define DATA_INTERFACE_MACESHARED_EXPORT __declspec(dllexport)
#  endif
#else
#  define DATA_INTERFACE_MACESHARED_EXPORT
#endif

#endif // DATA_INTERFACE_MACE_GLOBAL_H
