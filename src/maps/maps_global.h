#ifndef MAPS_GLOBAL_H
#define MAPS_GLOBAL_H

#ifdef _MSC_VER
#  if defined(MAPS_LIBRARY)
#    define MAPSSHARED_EXPORT __declspec(dllexport)
#  else
#    define MAPSSHARED_EXPORT __declspec(dllexport)
#  endif
#else
#  define DATA_GENERIC_COMMAND_ITEMSHARED_EXPORT
#endif

#endif // MAPS_GLOBAL_H
