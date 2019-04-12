#ifndef BASE_TOPIC_GLOBAL_H
#define BASE_TOPIC_GLOBAL_H

#ifdef _MSC_VER
#  if defined(BASE_TOPIC_LIBRARY)
#    define BASE_TOPICSHARED_EXPORT  __declspec(dllexport)
#  else
#    define BASE_TOPICSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define BASE_TOPICSHARED_EXPORT
#endif


#endif // BASE_TOPIC_GLOBAL_H
