#ifndef MODULE_ROS_GLOBAL_H
#define MODLUE_ROS_GLOBAL_H

#ifdef _MSC_VER
#  if defined(MODULE_ROS_LIBRARY)
#    define MODULE_ROSSHARED_EXPORT  __declspec(dllexport)
#  else
#    define MODULE_ROSSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define MODULE_ROSSHARED_EXPORT
#endif

#endif // MODULE_ROS_GLOBAL_H
