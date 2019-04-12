#ifndef MACE_CORE_GLOBAL_H
#define MACE_CORE_GLOBAL_H

#ifdef _MSC_VER
#  if defined(MACE_CORE_LIBRARY)
#    define MACE_CORESHARED_EXPORT  __declspec(dllexport)
#  else
#    define MACE_CORESHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define MACE_CORESHARED_EXPORT
#endif


#ifdef __GNUC__
#define DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif


#endif // MACE_CORE_GLOBAL_H
