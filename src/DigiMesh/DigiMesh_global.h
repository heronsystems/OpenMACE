#ifndef DIGIMESH_GLOBAL_H
#define DIGIMESH_GLOBAL_H

#ifdef _MSC_VER
#if defined(DIGIMESH_LIBRARY)
#  define DIGIMESHSHARED_EXPORT __declspec(dllexport)
#else
#  define DIGIMESHSHARED_EXPORT __declspec(dllimport)
#endif
#else
#  define DIGIMESHSHARED_EXPORT
#endif

#endif // DIGIMESH_GLOBAL_H
