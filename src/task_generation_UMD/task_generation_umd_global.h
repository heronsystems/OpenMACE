#ifndef TASK_GENERATION_UMD_GLOBAL_H
#define TASK_GENERATION_UMD_GLOBAL_H

#ifdef _MSC_VER
#  if defined(TASK_GENERATION_UMD_LIBRARY)
#    define TASK_GENERATION_UMDSSHARED_EXPORT  __declspec(dllexport)
#  else
#    define TASK_GENERATION_UMDSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define TASK_GENERATION_UMDSHARED_EXPORT
#endif


#endif // TASK_GENERATION_UMD_GLOBAL_H
