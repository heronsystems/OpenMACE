#ifndef OCCUPANCY_MAP_2D_INFLATED_H
#define OCCUPANCY_MAP_2D_INFLATED_H

#include <map>

#include "base/pose/cartesian_position_2D.h"

#include "data_2d_grid.h"
#include "occupancy_definition.h"

#include "iterators/circle_map_iterator.h"

namespace mace {
namespace maps {

enum OccupancyInflationMetric {
    EMPTY,
    TRUE_OCCUPANCY,
    INFLATION_OCCUPANCY,
    NOT_OCCUPIED
};

struct OccupancyInflationStructure
{
        unsigned int count;
        OccupancyInflationMetric metric;
};

class OccupancyMap_InflationParameters
{
public:
    OccupancyMap_InflationParameters(const double &inflationRadius = 0.0);

public:
    double getInflationRadius() const
    {
        return this->radius;
    }

    void setInflationRadius(const double &inflationRadius, const bool &updateMap)
    {
        UNUSED(updateMap);
        this->radius = inflationRadius;
    }

public:
    OccupancyMap_InflationParameters& operator = (const OccupancyMap_InflationParameters &rhs)
    {
        this->radius = rhs.radius;
        return *this;
    }

    bool operator == (const OccupancyMap_InflationParameters &rhs) const
    {
        if(this->radius != rhs.radius){
            return false;
        }
        return true;
    }

    bool operator != (const OccupancyMap_InflationParameters &rhs) const{
        return !(*this == rhs);
    }

private:
    double radius;
};

class OccupancyMap_2DInflated
{
public:
    OccupancyMap_2DInflated(const Data2DGrid<OccupiedResult>* originalMap, const OccupancyMap_InflationParameters &parameters);

private:
    void updateMapInflation(const std::map<unsigned int, OccupiedResult> &updates);

private:
    Data2DGrid<OccupancyInflationStructure>* inflatedMap;
    OccupancyMap_InflationParameters parameters;
};

} //end of namespace maps
} //end of namespace mace
#endif // OCCUPANCY_MAP_2D_INFLATED_H
