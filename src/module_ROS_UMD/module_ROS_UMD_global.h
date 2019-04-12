#ifndef MODULE_ROS_UMD_GLOBAL_H
#define MODLUE_ROS_UMD_GLOBAL_H

#ifdef _MSC_VER
#  if defined(MODULE_UMD_ROS_LIBRARY)
#    define MODULE_ROS_UMDSHARED_EXPORT  __declspec(dllexport)
#  else
#    define MODULE_ROS_UMDSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define MODULE_ROS_UMDSHARED_EXPORT
#endif

#endif // MODULE_ROS_UMD_GLOBAL_H
