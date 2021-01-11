#ifndef TRAJECTORY_CONTROL_GLOBAL_H
#define TRAJECTORY_CONTROL_GLOBAL_H

#ifdef _MSC_VER
#  if defined(TRAJECTORY_CONTROL_LIBRARY)
#    define TRAJECTORY_CONTROL_EXPORT  __declspec(dllexport)
#  else
#    define TRAJECTORY_CONTROL_EXPORT  __declspec(dllimport)
#  endif
#else
#  define TRAJECTORY_CONTROLSHARED_EXPORT
#endif

#endif // TRACJTORY_CONTROL_GLOBAL_H


