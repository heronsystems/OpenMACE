#ifndef MODULE_EXTERNAL_LINK_GLOBAL_H
#define MODULE_EXTERNAL_LINK_GLOBAL_H


#ifdef _MSC_VER
#  if defined(MODULE_EXTERNAL_LINK_LIBRARY)
#    define MODULE_EXTERNAL_LINKSHARED_EXPORT  __declspec(dllexport)
#  else
#    define MODULE_EXTERNAL_LINKSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define MODULE_EXTERNAL_LINKSHARED_EXPORT
#endif


#endif // MODULE_EXTERNAL_LINK_GLOBAL_H
