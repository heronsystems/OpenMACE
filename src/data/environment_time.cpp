#include "environment_time.h"

#include <iostream>

#include <cmath>

namespace  Data {

const int EnvironmentTime::NonLeapDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*!
 * \brief Default Constructor
 *
 * Set all feilds to zero
 */
EnvironmentTime::EnvironmentTime()
{
    year = 0;
    month = 0;
    dayOfMonth = 0;
    hour = 0;
    minute = 0;
    second = 0;
    millisecond = 0;
    microsecond = 0;
    instance = 0;
}

/*!
 * \brief Constructor
 *
 * takes in a pointer to a tm object, and the number of microseconds.
 * Fields are initialized from the fields of the tm struct, and the microseconds parameter.
 * \param t tm structure
 * \param microseconds number of microseconds for the timestamp.
 */
EnvironmentTime::EnvironmentTime(const tm &t, const int &microseconds)
{
    year = t.tm_year;
    month = t.tm_mon;
    dayOfMonth = t.tm_mday;
    hour = t.tm_hour;
    minute = t.tm_min;
    second = t.tm_sec;
    millisecond = microseconds/1000;
    microsecond = microseconds - 1000*millisecond;
    instance = 0;

    isDST = t.tm_isdst;
}

/*!
 * \brief Copy Constructor
 * \param that Time to copy from
 */
EnvironmentTime::EnvironmentTime(const EnvironmentTime& that)
{
    this->setTime(that);
}


/*!
 * \brief Assignment operator
 * \param rhs The right side of assignment
 */
EnvironmentTime& EnvironmentTime::operator= (const EnvironmentTime& rhs)
{
    this->setTime(rhs);
    return *this;
}


EnvironmentTime EnvironmentTime::operator +(const uint64_t &milliseconds) const
{
    QDateTime datetime;
    datetime.setMSecsSinceEpoch(0);
    QDate date;
    date.setDate(this->year, this->month, this->dayOfMonth);
    datetime.setDate(date);

    QTime time;
    time.setHMS(this->hour, this->minute, this->second, this->millisecond);
    datetime.setTime(time);
    datetime = datetime.addMSecs(milliseconds);

    Data::EnvironmentTime retval;
    retval.setTime(datetime);
    return retval;

}

EnvironmentTime &EnvironmentTime::operator +=(const uint64_t &milliseconds)
{
    QDateTime datetime;
    datetime.setMSecsSinceEpoch(0);
    QDate date;
    date.setDate(this->year, this->month, this->dayOfMonth);
    datetime.setDate(date);

    QTime time;
    time.setHMS(this->hour, this->minute, this->second, this->millisecond);
    datetime.setTime(time);
    datetime = datetime.addMSecs(milliseconds);

    this->setTime(datetime);
    return *this;
}

/*!
 * \brief Get the current time from a device shifted by an applied shift
 *
 * Shifted time is newTime = scale(actualTime + offset)
 * \param device Device to get time from
 * \param time time object to fill out
 */
void EnvironmentTime::CurrentTime(const Devices &device, EnvironmentTime &time)
{
    switch(device)
    {
    case SYSTEMCLOCK:
    {
        QDateTime CurrTime = QDateTime::currentDateTime();

        time.year = CurrTime.date().year();
        time.month = CurrTime.date().month();
        time.dayOfMonth = CurrTime.date().day();
        time.hour = CurrTime.time().hour();
        time.minute = CurrTime.time().minute();
        time.second = CurrTime.time().second();
        time.millisecond = CurrTime.time().msec();
        time.microsecond = 0;
        time.instance = 0;
        break;
    }
    default:
        throw;
    }
}

/*!
 * \brief EQ check operator
 * \param rhs Right hand side
 * \return true equivilant
 */
bool EnvironmentTime::operator == ( const EnvironmentTime& rhs ) const
{
    if(this->year != rhs.year)
        return false;
    if(this->month != rhs.month)
        return false;
    if(this->dayOfMonth != rhs.dayOfMonth)
        return false;
    if(this->hour != rhs.hour)
        return false;
    if(this->minute != rhs.minute)
        return false;
    if(this->second != rhs.second)
        return false;
    if(this->millisecond != rhs.millisecond)
        return false;
    if(this->microsecond != rhs.microsecond)
        return false;
    if(this->instance != rhs.instance)
        return false;

    return true;
}

bool EnvironmentTime::EQTimeComponents(const EnvironmentTime& rhs) const
{
    if(this->year != rhs.year)
        return false;
    if(this->month != rhs.month)
        return false;
    if(this->dayOfMonth != rhs.dayOfMonth)
        return false;
    if(this->hour != rhs.hour)
        return false;
    if(this->minute != rhs.minute)
        return false;
    if(this->second != rhs.second)
        return false;
    if(this->millisecond != rhs.millisecond)
        return false;
    if(this->microsecond != rhs.microsecond)
        return false;

    return true;
}

/*!
 * \brief NEQ check operator
 * \param rhs Right hand side
 * \return true if NOT equivilant
 */
bool EnvironmentTime::operator != ( const EnvironmentTime& rhs ) const
{
    return !(*this == rhs);
}

/*!
 * \brief Less than operator
 * \param rhs Right hand side of less than comparison
 * \return true is less than
 */
bool EnvironmentTime::operator < (const EnvironmentTime& rhs ) const
{
    if(this->year != rhs.year)
        return this->year < rhs.year;
    if(this->month != rhs.month)
        return this->month < rhs.month;
    if(this->dayOfMonth != rhs.dayOfMonth)
        return this->dayOfMonth < rhs.dayOfMonth;
    if(this->hour != rhs.hour)
        return this->hour < rhs.hour;
    if(this->minute != rhs.minute)
        return this->minute < rhs.minute;
    if(this->second != rhs.second)
        return this->second < rhs.second;
    if(this->millisecond != rhs.millisecond)
        return this->millisecond < rhs.millisecond;
    if(this->microsecond != rhs.microsecond)
        return this->microsecond < rhs.microsecond;
    if(this->instance != rhs.instance)
        return this->instance < rhs.instance;

    //equal, so not less than

    return false;
}

/*!
 * \brief Greater than operator
 * \param rhs Right hand side of greater than than comparison
 * \return true is greater than
 */
bool EnvironmentTime::operator > ( const EnvironmentTime& rhs ) const
{
    if(*this == rhs)
        return 1>1;
    else
        return *this >= rhs;
}

/*!
 * \brief Greater than equal to operator
 * \param rhs Right hand side of less than comparison
 * \return true is greater than or equal to
 */
bool EnvironmentTime::operator >= ( const EnvironmentTime& rhs ) const
{
    return !(*this < rhs);
}

/*!
 * \brief Subtraction operator
 *
 * Microsecond difference between the two times.
 * Performs subtraction from an internal epoch
 * \param rhs Right hand side of the subtraction
 * \return Microsecond difference between the two times
 */
uint64_t EnvironmentTime::operator -( const EnvironmentTime& rhs ) const
{
    uint64_t difference = 0;

    uint64_t Modifier = 1;
    difference += Modifier*(this->microsecond - rhs.microsecond);

    Modifier *= 1000;
    difference += Modifier*(this->millisecond - rhs.millisecond);

    Modifier *= 1000;
    difference += Modifier*(this->second - rhs.second);

    Modifier *= 60;
    difference += Modifier*(this->minute - rhs.minute);

    Modifier *= 60;
    difference += Modifier*(this->hour - rhs.hour);

    Modifier *= 24;
    difference += Modifier*(this->dayOfMonth - rhs.dayOfMonth);

    const uint64_t DailyMicroSeconds = 86400000000;

    //Determine if lhs is leap year
    bool isLHSLeapYear = false;
    if((this->year % 4 == 0 && this->year % 100 != 0) || this->year % 400 == 0)
        isLHSLeapYear = true;

    //Determine if RHS is leap year
    bool isRHSLeapYear = false;
    if((rhs.year % 4 == 0 && rhs.year % 100 != 0) || rhs.year % 400 == 0)
        isRHSLeapYear = true;

    int LHSDays = 0;
    for(int i = 0 ; i < this->month-1 ; i++)
    {
        LHSDays += NonLeapDays[i];
    }

    int RHSDays = 0;
    for(int i = 0 ; i < rhs.month-1 ; i++)
    {
        RHSDays += NonLeapDays[i];
    }


    if(isLHSLeapYear == true && this->month > 2)
        LHSDays++;

    if(isRHSLeapYear == true && rhs.month > 2)
        RHSDays++;


    uint64_t DeltaDays = LHSDays - RHSDays;

    difference += DeltaDays*DailyMicroSeconds;

    if(this->year < rhs.year)
    {
        for(int i = this->year ; i < rhs.year ; i++)
        {
            if((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
                difference -= 366*DailyMicroSeconds;
            else
                difference -= 365*DailyMicroSeconds;
        }
    }

    if(rhs.year < this->year)
    {
        for(int i = rhs.year ; i < this->year ; i++)
        {
            if((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
                difference += 366*DailyMicroSeconds;
            else
                difference += 365*DailyMicroSeconds;
        }
    }

    return difference;
}




/*!
 * \brief Set the time to the passed structure
 * \param t The structure to set the time to
 */
void EnvironmentTime::setTime(const EnvironmentTime &t)
{
    this->year = t.year;
    this->month = t.month;
    this->dayOfMonth = t.dayOfMonth;
    this->hour = t.hour;
    this->minute = t.minute;
    this->second = t.second;
    this->millisecond = t.millisecond;
    this->microsecond = t.microsecond;
    this->instance = t.instance;
}

void EnvironmentTime::setTime(const QDateTime &t)
{
    this->year = t.date().year();
    this->month = t.date().month();
    this->dayOfMonth = t.date().day();
    this->hour = t.time().hour();
    this->minute = t.time().minute();
    this->second = t.time().second();
    this->millisecond = t.time().msec();
}

/*!
 * \brief Returns the number of seconds from epoch
 * \return Number of seconds from epoch
 */
double EnvironmentTime::ToSecSinceEpoch() const
{
    QDateTime datetime;
    datetime.setMSecsSinceEpoch(0);
    QDate date;
    date.setDate(this->year, this->month, this->dayOfMonth);
    datetime.setDate(date);

    QTime time;
    time.setHMS(this->hour, this->minute, this->second, this->millisecond);
    datetime.setTime(time);

    //double secOfDay = (this->hour*60.0*60.0) + (this->minute*60.0) + ((double)this->second) + (this->millisecond/1000.0) + (this->microsecond/1000000.0);
    //time = time.addSecs(secOfDay);
    double secDateSinceEpoch = datetime.toMSecsSinceEpoch()/1000.0;
    return secDateSinceEpoch  + (this->microsecond/1000000.0);
}

//!
//! \brief Returns the number of milliseconds from epoch
//! \return Number of milleseconds from epoch
//!
double EnvironmentTime::ToMillisecondsSinceEpoch() const
{
    QDateTime datetime;
    datetime.setMSecsSinceEpoch(0);
    QDate date;
    date.setDate(this->year, this->month, this->dayOfMonth);
    datetime.setDate(date);

    QTime time;
    time.setHMS(this->hour, this->minute, this->second, this->millisecond);
    datetime.setTime(time);

    double mSecondsSinceEpoch = datetime.toMSecsSinceEpoch();
    return mSecondsSinceEpoch  + (this->microsecond/1000.0);
}

//!
//! \brief Construct time from seconds since epoch.
//! \param sec Seconds since epoch.
//! \return Time structure from given time
//!
EnvironmentTime EnvironmentTime::FromSecSinceEpoch(const double &sec)
{
    double sec_floored = std::floor(sec);
    double ms_floored = sec_floored*1000;

    QDateTime QTDateTime;
    QTDateTime.setMSecsSinceEpoch(ms_floored);

    double remainder_s = (sec - sec_floored);
    double remainder_ms = remainder_s*1000;
    double whole_ms = std::floor(remainder_ms);
    double remainder_us = (remainder_ms - whole_ms)*1000;

    EnvironmentTime time;
    time.year = QTDateTime.date().year();
    time.month = QTDateTime.date().month();
    time.dayOfMonth = QTDateTime.date().day();
    time.hour = QTDateTime.time().hour();
    time.minute = QTDateTime.time().minute();
    time.second = QTDateTime.time().second();
    time.millisecond = whole_ms;
    time.microsecond = std::floor(remainder_us);

    return time;
}

//!
//! \brief Construct time from seconds since epoch.
//! \param sec Seconds since epoch.
//! \return Time structure from given time
//!
EnvironmentTime EnvironmentTime::FromMillisecondsSinceEpoch(const double &msec)
{
    return FromSecSinceEpoch(msec/1000.0);
}


//!
//! \brief Convert the simulation time to a QDateTime object
//!
//! Note that QDateTime does not have support for microseconds
//! \return Resulting QDateTime object
//!
QDateTime EnvironmentTime::ToQTDateTime() const
{
    QDateTime datetime;
    datetime.setMSecsSinceEpoch(0);
    QDate date;
    date.setDate(this->year, this->month, this->dayOfMonth);
    datetime.setDate(date);

    QTime time;
    time.setHMS(this->hour, this->minute, this->second, this->millisecond);
    datetime.setTime(time);

    return datetime;
}

QString EnvironmentTime::ToString() const
{
    QString str = "";

    str += QString::number(this->year) + "." + QString::number(this->month) + "." + QString::number(this->dayOfMonth) + ":";
    str += QString::number(this->hour) + "." + QString::number(this->minute) + "." + QString::number(this->second) + "." + QString::number(this->millisecond) + "." + QString::number(this->microsecond);

    return str;
}

std::string EnvironmentTime::dateString() const
{
    std::string str = "";
    str += std::to_string(this->year) + "_" + std::to_string(this->month) + "_" + std::to_string(this->dayOfMonth);
    return str;
}

}
