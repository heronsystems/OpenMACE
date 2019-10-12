#include "map_cell.h"

#include <math.h>

namespace mace{
namespace maps{

//!
//! \brief MapCell Default constructor
//!
MapCell::MapCell() :
    occupiedResult(mace::maps::OccupiedResult::UNKNOWN), logOddsProbability(0.0), potentialTaskFlag(false)
{
    updatedAtTime = std::make_shared<Data::EnvironmentTime>();
}

//!
//! \brief MapCell Constructor
//! \param cellValue Default occupied result for the cell
//!
MapCell::MapCell(mace::maps::OccupiedResult occupiedResult) :
    occupiedResult(occupiedResult), logOddsProbability(0.0), potentialTaskFlag(false)
{
    updatedAtTime = std::make_shared<Data::EnvironmentTime>();
}

//!
//! \brief MapCell Constructor
//! \param cellValue Default occupied result for the cell
//! \param logOddsProbability Initial log-odds probability value.
//!
MapCell::MapCell(mace::maps::OccupiedResult occupiedResult, double confidence) :
    occupiedResult(occupiedResult), logOddsProbability(confidence), potentialTaskFlag(false)
{
    updatedAtTime = std::make_shared<Data::EnvironmentTime>();
}

//!
//! \brief MapCell Constructor
//! \param cellValue Default occupied result for the cell
//! \param logOddsProbability Initial log-odds probability value.
//! \param potentialTaskFlag Initial task flag. True if this cell should trigger a task, false otherwise.
//!
MapCell::MapCell(mace::maps::OccupiedResult occupiedResult, double confidence, bool potentialTaskFlag) :
    occupiedResult(occupiedResult), logOddsProbability(confidence), potentialTaskFlag(potentialTaskFlag)
{
    updatedAtTime = std::make_shared<Data::EnvironmentTime>();
}

//!
//! \brief updateLogOddsProbability Update the cell's log-odds probabilty value
//! \param occupiedResult Measured occupied result from the sensor
//! \param p_d Sensor's probability of detection value
//! \param p_fa Sensor's probability of false alarm value
//! \param sigma Confidence in sensor reading (e.g. attenuation based on distance from the sensor origin)
//!
void MapCell::updateLogOddsProbability(const OccupiedResult &occupiedResult, const double &p_d, const double &p_fa, const double &sigma) {
    double logOdds = 0.0;

    if(occupiedResult == OccupiedResult::OCCUPIED) {
        logOdds = p_d; // logOdds_occupied
    }
    else if(occupiedResult == OccupiedResult::NOT_OCCUPIED) {
        logOdds = -p_fa; // logOdds_free
    }
    else if(occupiedResult == OccupiedResult::UNKNOWN) {
        logOdds = 0.0;
    }
    else if(occupiedResult == OccupiedResult::ENVIRONMENT_BOUNDARY) {
        logOdds = 0.0;
    }

    // Calculate log odds:
    logOdds = this->getLogOdds() + logOdds*sigma;

    // Update our confidence with the new value:
    logOddsProbability = logOdds;

    // TODO: Either here or in the sensors module:
    //          - Update potential Task flag based on result and log odds value
}

} //end of namespace maps
} //end of namespace mace
