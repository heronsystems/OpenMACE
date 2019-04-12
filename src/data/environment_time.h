#ifndef ENVIRONMENT_TIME_H
#define ENVIRONMENT_TIME_H

#include <QDateTime>
#include <QDataStream>
#include <QMetaType>

#include "data_global.h"

#include <cmath>

namespace Data
{

/*!
 * \brief Enumeration of time sources availabe
 */
enum Devices
{
    SYSTEMCLOCK //! Use the system clock, requires outside synchronization
};

/*!
 * \brief The object holding time in the simulation.
 *
 * This object will likely not appropriatly handle a DST event.
 */
class DATASHARED_EXPORT EnvironmentTime
{
public:

    /*!
     * \brief Default Constructor
     *
     * Set all fields to zero
     */
    EnvironmentTime();


    /*!
     * \brief Copy Constructor
     * \param that Time to copy from
     */
    EnvironmentTime(const EnvironmentTime& that);


    /*!
     * \brief Constructor
     *
     * takes in a pointer to a tm object, and the number of microseconds.
     * Fields are initialized from the fields of the tm struct, and the microseconds parameter.
     * \param t tm structure
     * \param microseconds number of microseconds for the timestamp.
     */
    EnvironmentTime(const tm &t, const int &microseconds);


    /*!
     * \brief Assignment operator
     * \param rhs The right side of assignment
     */
    EnvironmentTime& operator= (const EnvironmentTime& rhs);


    /*!
     * \brief Get the current time from a device shifted by an applied shift
     *
     * Shifted time is newTime = scale(actualTime + offset)
     * \param device Device to get time from
     * \param time time object to fill out
     */
    static void CurrentTime(const Devices &device, EnvironmentTime &time);

    /*!
     * \brief EQ check operator
     * \param rhs Right hand side
     * \return true equivilant
     */
    bool operator == ( const EnvironmentTime& rhs ) const;


    //!
    //! \brief Method to determine if two times are the same time components, this method ingores instance
    //! \param rhs Right hand side
    //! \return true if time components are equal
    //!
    bool EQTimeComponents(const EnvironmentTime& rhs) const;


    /*!
     * \brief NEQ check operator
     * \param rhs Right hand side
     * \return true if NOT equivilant
     */
    bool operator != ( const EnvironmentTime& rhs ) const;

    /*!
     * \brief Less than operator
     * \param rhs Right hand side of less than comparison
     * \return true is less than
     */
    bool operator < ( const EnvironmentTime& rhs ) const;

    /*!
     * \brief Greater than operator
     * \param rhs Right hand side of greater than than comparison
     * \return true is greater than
     */
    bool operator > ( const EnvironmentTime& rhs ) const;

    /*!
     * \brief Greater than equal to operator
     * \param rhs Right hand side of greater than or equal to comparison
     * \return true is greater than or equal to
     */
    bool operator >= ( const EnvironmentTime& rhs ) const;


    /*!
     * \brief Subtraction operator
     *
     * Microsecond difference between the two times.
     * Performs subtraction from an internal epoch
     * \param rhs Right hand side of the subtraction
     * \return Microsecond difference between the two times
     */
    uint64_t operator -( const EnvironmentTime& microseconds ) const;


    //!
    //! \brief operator +
    //! \param milliseconds
    //! \return
    //!
    EnvironmentTime operator +(const uint64_t &milliseconds) const;

    //!
    //! \brief operator +=
    //! \param milliseconds
    //! \return
    //!
    EnvironmentTime &operator +=(const uint64_t &milliseconds);



//    /*!
//     * \brief Addition operator
//     *
//     * Microsecond addition between the two times.
//     * Performs addition from an internal epoch
//     * \param rhs Right hand side of the addition
//     * \return Microsecond addition between the two times
//     */
//    uint64_t operator +( const SimulationTime& microseconds ) const;


    /*!
     * \brief Set the time to the passed structure
     * \param t The structure to set the time to
     */
    void setTime(const EnvironmentTime &t);

    //!
    //! \brief setTime
    //! \param t
    //!
    void setTime(const QDateTime &t);

    /*!
     * \brief Convert the time to a string
     * \return The time in string form
     */
    QString ToString() const;


    //!
    //! \brief dateString
    //! \return
    //!
    std::string dateString() const;

    /*!
     * \brief Returns the number of seconds from epoch
     * \return Number of seconds from epoch
     */
    double ToSecSinceEpoch() const;

    //!
    //! \brief Returns the number of milliseconds from epoch
    //! \return Number of milleseconds from epoch
    //!
    double ToMillisecondsSinceEpoch() const;

    //!
    //! \brief Construct time from seconds since epoch.
    //! \param sec Seconds since epoch.
    //! \return Time structure from given time
    //!
    static EnvironmentTime FromSecSinceEpoch(const double &sec);

    //!
    //! \brief Construct time from milliseconds since epoch.
    //! \param sec Milliseconds since epoch.
    //! \return Time structure from given time
    //!
    static EnvironmentTime FromMillisecondsSinceEpoch(const double &msec);

    //!
    //! \brief Convert the simulation time to a QDateTime object
    //!
    //! Note that QDateTime does not have support for microseconds
    //! \return Resulting QDateTime object
    //!
    QDateTime ToQTDateTime() const;

public:

    //! Years since 1900
    int year;

    //! The current month.  0 = January, 1 = February, etc.
    int month;

    //! The current day of the month.  (Between 1 and 31.)
    int dayOfMonth;

    //! The current hour after midnight.  (Between 0 and 23.)
    int hour;

    //! The current minute after the hour.  (Between 0 and 59.)
    int minute;

    //! The current second after the minute.  (Between 0 and 59.)
    int second;

    //! The current millisecond after the second.  (Between 0 and 999.)
    int millisecond;

    //! The current microsecond after the millisecond.  (Between 0 and 999).
    int microsecond;

    //! Final catch-all to distinguish time instances that excede resolution of timing device
    uchar instance;

    //! Boolean to indicate if it is DST
    bool isDST;

private:

    static const int NonLeapDays[];

};

} // Data namespace

Q_DECLARE_METATYPE(Data::EnvironmentTime)

#endif // TIME_H
