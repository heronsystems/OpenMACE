#ifndef OBSERVATION_HISTORY_H
#define OBSERVATION_HISTORY_H

#include "common/common.h"

#include <string>
#include <map>
#include <vector>
#include <thread>
#include <mutex>

#include "mace_core_global.h"


typedef int T;

namespace MaceCore
{

enum Observation_Status
{

    Observation_Empty,
    Observation_Bad_Request,
    Observation_Complete

};

/*!
 * \brief Enumeration defining direction which can be searched in the history
 */
enum SearchDirection
{
    FORWARD_INCLUSION, //! Search foward in time, including the time passed
    BACKWARD_EXCLUSION, //! Search backward in time, excluding the time passed
};

//! A time history of observations
/*!
 *  This object is used to manage a time history of observations of some form of data
 *  Each observation has a time which it was taken.
 *  This time is used to index the observation in a QMap datastructure.
 *
 *  When an observation is no longer usefull it is written to an underlaying stream and removed from the object.
 *  This helps keep the data contained in this object small which speeds up searching.
 */
template <class T, class D> class ObservationHistory
{
public:

    //! Constructor
    /*!
      Opens the given filename and sets as stream to write data to
      \param millisecondToKeep How many milliseconds of history to keep
   */
    ObservationHistory(uint64_t millisecondToKeep)
    {
        this->m_MillisecondsToKeep = millisecondToKeep;
    }

    ObservationHistory(const ObservationHistory &that) :
        m_MillisecondsToKeep(that.m_MillisecondsToKeep),
        m_CurrentMapFull(that.m_CurrentMapFull)
    {

    }

    //! Destructor
    /*!
   * Write all data in current tree into the stream then delete it all.
   */
    ~ObservationHistory()
    {
        //lock the mutex
        std::lock_guard<std::mutex> guard(m_Mutex);
        UNUSED(guard);

        typename std::map<T, D>::iterator KeyValueIterationFull;

        //delete each value in the map
        KeyValueIterationFull=m_CurrentMapFull.begin();
        while(KeyValueIterationFull!=m_CurrentMapFull.end())
        {
            //T* tmpV = KeyValueIterationFull.value();
            KeyValueIterationFull = this->m_CurrentMapFull.erase(KeyValueIterationFull);
            //delete tmpV;

        } //End of while loop

    } // End of Destructor


    //! Insert a new observation into the history
    /*!
     * If there exists any observations which are sufficiantly far from the time passed to this method
     * they are written to the stream then deleted
     * \param time Time of observation.
     * \param observation The observation inserting.
     */
    Observation_Status InsertObservation(const T &time, const D &observation)
    {
        //lock the mutex
        std::lock_guard<std::mutex> guard(m_Mutex);
        UNUSED(guard);

        typename std::map<T, D>::iterator KeyValueIterationFull;

        this->m_CurrentMapFull.insert({time, observation});


        //determine what time to look for in key history and delete everythig with values less than this
        //T Time_Prior_To_Erase = time - 1000*m_MillisecondsToKeep;

        //point the iterator to the first key in MAP
        KeyValueIterationFull=m_CurrentMapFull.begin();

        //This while loop searches through MAP to find times that are invalid by iterating KeyValueIterationFull
        //If the time at location is < earliest time delete it
        while(KeyValueIterationFull != m_CurrentMapFull.end())
        {

            //remove the element if its time is prior to the threshold
            typename std::map<T, D>::iterator lastItem = (m_CurrentMapFull.end()--);
            const int64_t TimeDelta = lastItem->first - KeyValueIterationFull->first;
            if(TimeDelta > 0 && ((uint64_t)TimeDelta) > 1000*m_MillisecondsToKeep)
            {

                //T* tmpV = KeyValueIterationFull.value();
                KeyValueIterationFull = this->m_CurrentMapFull.erase(KeyValueIterationFull);
                //delete tmpV;
            }
            else
            {
                //If the first is within the range, then all are
                break;
            }

        }

        //emit NewObservation(time, observation); //what is this function doing???

        return(Observation_Complete);

    } //End of InsertObservation Function


    //! Get all observations within a range of the passed millisecond range.
    /*!
     * \param centerTime Center of range requesting observations in.
     * \param millisecondRange Range of the observation request.
     * \param rangedTimes Set to include the observations inside the time range described in the parameters passed.
     */
    Observation_Status Get_Observations_Range(const T &centerTime, const T &timeRange, std::map<T, D*> &QMap_Modify)
    {
        /*
        if(m_CurrentMapFull.empty()==true)
        {
            return(Observation_Empty);
        }
        //lock the mutex
        std::lock_guard<std::mutex> guard(m_Mutex);
        UNUSED(guard);
        typename QMap<T, T*>::iterator KeyValueIterationFull;
        typename QMap<T, T*>::iterator KeyValueIterationBeg;
        typename QMap<T, T*>::iterator KeyValueIterationEnd;
        //find the beginning of the dataset of interest (subset will >= value)
        T begin_capture_dead = centerTime - timeRange;
        KeyValueIterationBeg=m_CurrentMapFull.lowerBound(begin_capture_dead);
        //find the end of the dataset of interest (subset will <= value)
        T end_capture_dead = centerTime + timeRange;
        KeyValueIterationEnd=m_CurrentMapFull.upperBound(end_capture_dead) - 1;
        //KeyValueIterationEnd--;
        //find the ending key value relating to subset period of interest and assign subset MAP values
        KeyValueIterationFull=KeyValueIterationBeg;
        while(KeyValueIterationFull!=KeyValueIterationEnd)
        {
            QMap_Modify.insert(KeyValueIterationFull.key(),KeyValueIterationFull.value());
            KeyValueIterationFull++;
        } //End of while loop inserting into new funciton
        */
        return(Observation_Complete);

    } //End of Get_Observations_Range Function


    /*!
     * \brief Get the closest observation to a specific time
     * \author Madison Blake
     * \param[in] time time to look for
     * \param[in] direction direction to go, false indicates backward in time, true indicates foward
     * \param[in] N number of observations to get
     * \param[out] observation List of observations
     * \param[out] times List of times for associated observations
     */
    void GetClosestObservation(const T &time, const SearchDirection &direction, const int &N, std::vector<D> &Observations, std::vector<T> &times) const
    {
        //lock the mutex
        std::lock_guard<std::mutex> guard(m_Mutex);
        UNUSED(guard);

        //ensure list is clear
        Observations.clear();
        times.clear();

        if(m_CurrentMapFull.size() == 0)
            return;

        //Get an iterator to the closest greater than our target time
        typename std::map<T, D>::const_iterator iterator = m_CurrentMapFull.lower_bound(time);

        //number of observations found
        int observationsFound = 0;

        //going foward
        if(direction == FORWARD_INCLUSION)
        {
            typename std::map<T, D>::const_iterator last_element = (m_CurrentMapFull.end()--);
            //if target time is equal to end then there is only one.
            if(time == last_element->first)
            {
                Observations.push_back(last_element->second);
                times.push_back(last_element->first);
                observationsFound++;
                return;
            }

            //append until we reach the end or the target number of observations
            while( observationsFound < N && iterator != m_CurrentMapFull.end() )
            {
                Observations.push_back(iterator->second);
                times.push_back(iterator->first);
                iterator++;
                observationsFound++;
            }
        }

        //going backward
        if(direction == BACKWARD_EXCLUSION)
        {

            //if the time is previous to first time then there is no valid entry
            if(time < m_CurrentMapFull.begin()->first)
                return;

            // we are either above the target or at the target.
            // so decreiment to get bellow it (but check to ensure you can decriment)
            if(iterator == m_CurrentMapFull.begin())
                return;
            iterator--;

            //Continue appending until we reach the begening or the target number of observations
            while(observationsFound < N)
            {

                Observations.push_back(iterator->second);
                times.push_back(iterator->first);

                if(iterator == m_CurrentMapFull.begin())
                    break;

                iterator--;
                observationsFound++;
            }
        }
    }

    //! Get the previous N observations
    /*!
     * \brief GetNumberObservations This function grabs a specified amount of observations from the masters history
     * \param Number_of_Observations The number of observations desired for observance
     * \param QMap_Modify An empty map to return the values in
     */
    Observation_Status GetPreviousNumberObservations(const int Number_of_Observations, std::vector<D*> &Observations)
    {

        std::lock_guard<std::mutex> guard(m_Mutex);
        UNUSED(guard);

        Observations.clear();

        //Get an iterator to the end of the map
        typename std::map<T, D*>::iterator iterator = m_CurrentMapFull.end();

        //number of observations found
        int observationsFound = 0;

        if(m_CurrentMapFull.empty()==true)
        {
            return(Observation_Empty);
        }

        else if(m_CurrentMapFull.size()<Number_of_Observations)
        {

            while( observationsFound < Number_of_Observations && iterator != m_CurrentMapFull.end() )
            {
                Observations.append(iterator.value());
                iterator++;
                observationsFound++;
            }

            return(Observation_Bad_Request);
        }

        else
        {
            while( observationsFound < Number_of_Observations && iterator != m_CurrentMapFull.end() )
            {
                Observations.append(iterator.value());
                iterator++;
                observationsFound++;
            }

            return(Observation_Complete);
        }


    } //end of GetNumberObservations Function


    //! This simple function checks the size of the MAP. This can be used to check before going into deadreckoning or performing operations on MAP
    /*!
     * \brief Check_MAP_Size
     * \return The return is an integer that describes the MAP size
     */
    int Check_MAP_Size()
    {

        std::lock_guard<std::mutex> guard(m_Mutex);
        UNUSED(guard);

        int Size_MAP=0;
        Size_MAP=m_CurrentMapFull.size();
        return(Size_MAP);

    } // End of Check_Map_Function

private:

    //! The current full observation list
    std::map<T, D> m_CurrentMapFull;

    //! Mutex to protect against insertions to this datastructure.
    mutable std::mutex m_Mutex;

    //! The amount of time each observation is allowed to remain in the current list
    uint64_t m_MillisecondsToKeep;


};

} // MaceCore namespace

#endif // OBSERVATION_HISTORY_H
