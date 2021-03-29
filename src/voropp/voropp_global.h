#ifndef VOROPP_GLOBAL_H
#define VOROPP_GLOBAL_H

#ifdef _MSC_VER
#  if defined(VOROPP_LIBRARY)
#    define VOROPPSHARED_EXPORT  __declspec(dllexport)
#  else
#    define VOROPPSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define VOROPPSHARED_EXPORT
#endif

#endif // VOROPP_GLOBAL_H
