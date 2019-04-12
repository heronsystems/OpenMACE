#ifndef COMMSMAVLINK_GLOBAL_H
#define COMMSMAVLINK_GLOBAL_H

#ifdef _MSC_VER
#  if defined(COMMSMAVLINK_LIBRARY)
#    define COMMSMAVLINKSHARED_EXPORT  __declspec(dllexport)
#  else
#    define COMMSMAVLINKSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define COMMSMAVLINKSHARED_EXPORT
#endif

#endif // COMMSMAVLINK_GLOBAL_H
