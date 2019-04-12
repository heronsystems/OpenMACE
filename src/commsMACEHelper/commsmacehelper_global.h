#ifndef COMMSMACEHELPER_GLOBAL_H
#define COMMSMACEHELPER_GLOBAL_H

#ifdef _MSC_VER
#  if defined(COMMSMACEHELPER_LIBRARY)
#    define COMMSMACEHELPERSHARED_EXPORT  __declspec(dllexport)
#  else
#    define COMMSMACEHELPERSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define COMMSMACEHELPERSHARED_EXPORT
#endif

#endif // COMMSMACEHELPER_GLOBAL_H
