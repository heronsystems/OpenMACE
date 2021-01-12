#ifndef VEHICLE_PATH_LINEAR_H
#define VEHICLE_PATH_LINEAR_H

#include <string>
#include <thread>
#include <iostream>
#include "abstract_vehicle_path.h"

#include "base/pose/geodetic_position_3D.h"

using namespace mace;

class VehiclePath_Linear : public AbstractVehiclePath
{
public:

    VehiclePath_Linear();

    VehiclePath_Linear(const VehiclePath_Linear &copy);

    ~VehiclePath_Linear() = default;

public:

    void setVertices(const std::vector<pose::GeodeticPosition_3D> &vertices);

    std::vector<pose::GeodeticPosition_3D> getVertices() const;
    
private:
    std::vector<pose::GeodeticPosition_3D> m_vertices;
    
};

#endif // VEHICLE_PATH_H
