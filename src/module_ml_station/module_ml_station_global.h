#ifndef MODULE_ML_STATION_GLOBAL_H
#define MODULE_ML_STATION_GLOBAL_H

#ifdef _MSC_VER
#  if defined(MODULE_ML_STATION_LIBRARY)
#    define MODULE_ML_STATIONSHARED_EXPORT  __declspec(dllexport)
#  else
#    define MODULE_ML_STATIONSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define MODULE_ML_STATIONSHARED_EXPORT
#endif

#endif // MODULE_ML_STATION_GLOBAL_H
