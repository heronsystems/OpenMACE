#include "data_interpolation.h"

DataInterpolation::DataInterpolation() :
    MaceCore::MaceData()
{

}

DataInterpolation::DataInterpolation(uint64_t historyToKeepInms) :
    MaceCore::MaceData(historyToKeepInms)
{

}


MaceCore::VectorDynamics DataInterpolation::Fuse_VehicleDynamics(const MaceCore::TIME &time, const MaceCore::VectorDynamics &v0, const MaceCore::TIME &t0, const MaceCore::VectorDynamics &v1, const MaceCore::TIME &t1) const
{
    return v1;
}


MaceCore::VehicleLife DataInterpolation::Fuse_VehicleLife(const MaceCore::TIME &time, const MaceCore::VehicleLife &v0, const MaceCore::TIME &t0, const MaceCore::VehicleLife &v1, const MaceCore::TIME &t1) const
{
    return v1;
}

