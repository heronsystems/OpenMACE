#ifndef DATA_GLOBAL_H
#define DATA_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_LIBRARY)
#    define DATASHARED_EXPORT  __declspec(dllexport)
#  else
#    define DATASHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define DATASHARED_EXPORT
#endif

#endif //DATA_GLOBAL_H
