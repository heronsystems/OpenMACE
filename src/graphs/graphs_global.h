#ifndef GRAPHS_GLOBAL_H
#define GRAPHS_GLOBAL_H

#ifdef _MSC_VER
#  if defined(GRAPHS_LIBRARY)
#    define GRAPHSSHARED_EXPORT  __declspec(dllexport)
#  else
#    define GRAPHSSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define GRAPHSSHARED_EXPORT
#endif

#endif // GRAPHS_GLOBAL_H
