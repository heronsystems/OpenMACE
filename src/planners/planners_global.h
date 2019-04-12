#ifndef PLANNERS_GLOBAL_H
#define PLANNERS_GLOBAL_H

#ifdef _MSC_VER
#  if defined(PLANNERS_LIBRARY)
#    define PLANNERSSHARED_EXPORT  __declspec(dllexport)
#  else
#    define PLANNERSSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define PLANNERSSHARED_EXPORT
#endif

#endif // PLANNERS_GLOBAL_H
