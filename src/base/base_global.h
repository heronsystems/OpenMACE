#ifndef BASE_GLOBAL_H
#define BASE_GLOBAL_H

#ifdef _MSC_VER
#  if defined(BASE_LIBRARY)
#    define BASESHARED_EXPORT  __declspec(dllexport)
#  else
#    define BASESHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define BASESHARED_EXPORT
#endif

#endif // BASE_GLOBAL_H
