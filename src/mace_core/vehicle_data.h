#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include "common/optional_parameter.h"

namespace MaceCore
{

typedef int TIME;

class VehicleLife
{
public:
    double batteryPercent;
    OptionalParameter<double> flightTimeInSec;
    OptionalParameter<double> flightDistanceInMeters;
};


class VectorDynamics
{
public:

};


class FullVehicleDynamics
{
public:
    VectorDynamics positional;
    VectorDynamics attitude;
};

} //End MaceCore Namespace

#endif // VEHICLE_DATA_H
