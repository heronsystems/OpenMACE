#ifndef DATA_INTERPOLATION_H
#define DATA_INTERPOLATION_H

#include "mace_core/mace_data.h"

class DataInterpolation : public MaceCore::MaceData
{
public:
    DataInterpolation();


    DataInterpolation(uint64_t historyToKeepInms);


    virtual MaceCore::VectorDynamics Fuse_VehicleDynamics(const MaceCore::TIME &time, const MaceCore::VectorDynamics &v0, const MaceCore::TIME &t0, const MaceCore::VectorDynamics &v1, const MaceCore::TIME &t1) const;


    virtual MaceCore::VehicleLife Fuse_VehicleLife(const MaceCore::TIME &time, const MaceCore::VehicleLife &v0, const MaceCore::TIME &t0, const MaceCore::VehicleLife &v1, const MaceCore::TIME &t1) const;

};

#endif // DATA_INTERPOLATION_H
