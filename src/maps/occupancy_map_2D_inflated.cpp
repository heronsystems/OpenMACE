#include "occupancy_map_2D_inflated.h"

namespace mace {
namespace maps {

OccupancyMap_InflationParameters::OccupancyMap_InflationParameters(const double &inflationRadius)
{
    this->radius = inflationRadius;
}

OccupancyMap_2DInflated::OccupancyMap_2DInflated(const Data2DGrid<OccupiedResult> *originalMap, const OccupancyMap_InflationParameters &parameters)
{
    UNUSED(originalMap);
    this->parameters = parameters;
}

void OccupancyMap_2DInflated::updateMapInflation(const std::map<unsigned int, OccupiedResult> &updates)
{
    std::map<unsigned int, OccupiedResult>::const_iterator it;

    //if the radius is smaller than the resolution of the map, there would be no updates to the may
    //thus this check ensures that this update is not consuming processor time for no use
    if((parameters.getInflationRadius() < inflatedMap->getXResolution()) && (parameters.getInflationRadius() < inflatedMap->getYResolution()))
        return;

    for(it = updates.begin(); it != updates.end(); ++it)
    {
        CircleMapIterator circleIt(this->inflatedMap,it->first,parameters.getInflationRadius());
        //we can break the if/else conditional first outside of the loop to avoid checking
        //the same condition inside the for loop every time
        if(it->second == OccupiedResult::OCCUPIED)
        {
            for(;!circleIt.isPastEnd();++circleIt)
            {
                //const unsigned int indexPosition = *circleIt;
                OccupancyInflationStructure* ptr = this->inflatedMap->getCellByIndex(*circleIt);
                ptr->metric = OccupancyInflationMetric::INFLATION_OCCUPANCY;
                ptr->count++;
            }
            //this avoids having an if else statement every loop iteration and we know we need to
            //mark the cell as being actually occupied
            OccupancyInflationStructure* ptr = this->inflatedMap->getCellByIndex(it->first);
            ptr->metric = OccupancyInflationMetric::TRUE_OCCUPANCY;

        }
        else if(it->second == OccupiedResult::NOT_OCCUPIED)
        {
            for(;!circleIt.isPastEnd();++circleIt)
            {
                //KEN: Check this
                //const unsigned int indexPosition = *circleIt;
//                OccupancyInflationStructure* ptr = this->inflatedMap->getCellByIndex(*circleIt);
                //*ptr = (*ptr <= 0) ? 0 : *ptr--;
            }
            //Since we know this explicit cell is not occupied, we can mark it as such and clear count
            OccupancyInflationStructure* ptr = this->inflatedMap->getCellByIndex(it->first);
            ptr->count = 0;
            ptr->metric = OccupancyInflationMetric::NOT_OCCUPIED;
        }

    }
}

} //end of namespace maps
} //end of namespace mace
