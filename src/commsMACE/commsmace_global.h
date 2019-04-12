#ifndef COMMSMACE_GLOBAL_H
#define COMMSMACE_GLOBAL_H

#ifdef _MSC_VER
#  if defined(COMMSMACE_LIBRARY)
#    define COMMSMACESHARED_EXPORT  __declspec(dllexport)
#  else
#    define COMMSMACESHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define COMMSMACESHARED_EXPORT
#endif

#endif // COMMSMACE_GLOBAL_H
