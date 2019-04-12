#ifndef COMMS_GLOBAL_H
#define COMMS_GLOBAL_H

#ifdef _MSC_VER
#  if defined(COMMS_LIBRARY)
#    define COMMSSHARED_EXPORT  __declspec(dllexport)
#  else
#    define COMMSSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define COMMSSHARED_EXPORT
#endif

#include <stdint.h>

#endif // COMMS_GLOBAL_H
