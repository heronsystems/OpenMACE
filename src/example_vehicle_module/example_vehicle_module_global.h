#ifndef EXAMPLE_VEHICLE_MODULE_GLOBAL_H
#define EXAMPLE_VEHICLE_MODULE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(EXAMPLE_VEHICLE_MODULE_LIBRARY)
#  define EXAMPLE_VEHICLE_MODULESHARED_EXPORT Q_DECL_EXPORT
#else
#  define EXAMPLE_VEHICLE_MODULESHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // EXAMPLE_VEHICLE_MODULE_GLOBAL_H
