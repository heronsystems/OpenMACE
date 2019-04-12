#ifndef MAP_CELL_H
#define MAP_CELL_H

#include <data/environment_time.h>
#include <maps/occupancy_definition.h>

#include <memory>

namespace mace {
namespace maps {

class MapCell
{
public:
    //!
    //! \brief MapCell Default constructor
    //!
    MapCell();

    //!
    //! \brief MapCell Constructor
    //! \param cellValue Default occupied result for the cell
    //!
    MapCell(mace::maps::OccupiedResult cellValue);

    //!
    //! \brief MapCell Constructor
    //! \param cellValue Default occupied result for the cell
    //! \param logOddsProbability Initial log-odds probability value.
    //!
    MapCell(mace::maps::OccupiedResult cellValue, double logOddsProbability);

    //!
    //! \brief MapCell Constructor
    //! \param cellValue Default occupied result for the cell
    //! \param logOddsProbability Initial log-odds probability value.
    //! \param potentialTaskFlag Initial task flag. True if this cell should trigger a task, false otherwise.
    //!
    MapCell(mace::maps::OccupiedResult cellValue, double logOddsProbability, bool potentialTaskFlag);

    //!
    //! \brief setCellValue Set the occupied result of the cell
    //! \param value Occupied result
    //!
    void setCellValue(const mace::maps::OccupiedResult &value) {
        this->occupiedResult = value;
    }

    //!
    //! \brief setCellValue Set the occupied result of the cell
    //! \param value Occupied result
    //! \param time Time at which the cell was last updated for decay calculations
    //!
    void setCellValue(const mace::maps::OccupiedResult &value, const Data::EnvironmentTime &time) {
        this->occupiedResult = value;
        updateCellUpdatedTime(time);
    }

    //!
    //! \brief getCellValue Get the occupied result of the cell
    //! \return Occupied result
    //!
    mace::maps::OccupiedResult getCellValue() { return occupiedResult; }

    //!
    //! \brief setLogOdds Set the log-odds probability
    //! \param logOdds Log-odds probability
    //!
    void setLogOdds(const double &logOdds) {
        this->logOddsProbability = logOdds;
    }

    //!
    //! \brief getLogOdds Get the current log-odds probability of the cell
    //! \return Log-odds probability
    //!
    double getLogOdds() { return this->logOddsProbability; }

    //!
    //! \brief setPotentialTaskFlag Set whether or not this cell should trigger a task
    //! \param taskFlag True if cell should trigger a task, false otherwise
    //!
    void setPotentialTaskFlag(const bool &taskFlag) {
        this->potentialTaskFlag = taskFlag;
    }

    //!
    //! \brief getPotentialTaskFlag Get current task flag
    //! \return True if cell should trigger a task, false otherwise
    //!
    bool getPotentialTaskFlag() { return potentialTaskFlag; }

    //!
    //! \brief updateCellUpdatedTime Update the time the cell was last updated
    //! \param time Time at which the cell was updated
    //!
    void updateCellUpdatedTime(const Data::EnvironmentTime &time) {
        this->updatedAtTime = std::make_shared<Data::EnvironmentTime>(time);
    }

    //!
    //! \brief getCellUpdatedTime Get the time at which the cell was last updated
    //! \return Time at which the cell was updated
    //!
    std::shared_ptr<Data::EnvironmentTime> getCellUpdatedTime() { return updatedAtTime; }

    //!
    //! \brief updateLogOddsProbability Update the cell's log-odds probabilty value
    //! \param occupiedResult Measured occupied result from the sensor
    //! \param p_d Sensor's probability of detection value
    //! \param p_fa Sensor's probability of false alarm value
    //! \param sigma Confidence in sensor reading (e.g. attenuation based on distance from the sensor origin)
    //!
    void updateLogOddsProbability(const OccupiedResult &occupiedResult, const double &p_d, const double &p_fa, const double &sigma);


private:
    //!
    //! \brief occupiedResult Map cell's current occupied/free status
    //!
    mace::maps::OccupiedResult occupiedResult;

    //!
    //! \brief logOddsProbability If positive, we are more confident in the cell value. If negative, we are less confident in the cell value.
    //!
    double logOddsProbability;

    //!
    //! \brief updatedAtTime Time at which the cell was last updated
    //!
    std::shared_ptr<Data::EnvironmentTime> updatedAtTime;

    //!
    //! \brief potentialTaskFlag True if cell should trigger a task, false otherwise
    //!
    bool potentialTaskFlag;

};

} //end of namespace maps
} //end of namespace mace

#endif // MAP_CELL_H
