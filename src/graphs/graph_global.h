#ifndef GRAPH_GLOBAL_H
#define GRAPH_GLOBAL_H

//#include <QtCore/qglobal.h>

#if defined(GRAPH_LIBRARY)
#  define GRAPHSHARED_EXPORT Q_DECL_EXPORT
#else
#  define GRAPHSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // GRAPH_GLOBAL_H
