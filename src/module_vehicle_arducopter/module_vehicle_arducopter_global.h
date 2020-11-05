#ifndef MODULE_VEHICLE_ARDUCOPTER_GLOBAL_H
#define MODULE_VEHICLE_ARDUCOPTER_GLOBAL_H


#ifdef _MSC_VER
#  if defined(MODULE_VEHICLE_ARDUCOPTER_LIBRARY)
#    define MODULE_VEHICLE_ARDUCOPTERSHARED_EXPORT  __declspec(dllexport)
#  else
#    define MODULE_VEHICLE_ARDUCOPTERSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define MODULE_VEHICLE_ARDUCOPTERSHARED_EXPORT
#endif


#endif // MODULE_VEHICLE_ARDUCOPTER_GLOBAL_H
