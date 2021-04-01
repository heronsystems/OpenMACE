#ifndef DATA_AUCTIONEER_GLOBAL_H
#define DATA_AUCTIONEER_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_AUCTIONEER_LIBRARY)
#    define DATA_AUCTIONEERSHARED_EXPORT __declspec(dllexport)
#  else
#    define DATA_AUCTIONEERSHARED_EXPORT __declspec(dllexport)
#  endif
#else
#  define DATA_AUCTIONEERSHARED_EXPORT
#endif

#endif // DATA_AUCTIONEER_GLOBAL_H
