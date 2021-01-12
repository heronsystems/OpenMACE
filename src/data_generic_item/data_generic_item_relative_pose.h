#ifndef DATA_GENERIC_ITEM_RELATIVEPOSE_H
#define DATA_GENERIC_ITEM_RELATIVEPOSE_H

#include <iostream>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

namespace DataGenericItem {

//! \class DataGenericItem_RelativePose Describes the position and orientation of an aircraft relative to some reference aircraft.
//! If this is the reference aircraft, it is expected that the position parameters contained here will be ignored.
class DataGenericItem_RelativePose
{
public:
    DataGenericItem_RelativePose();

    DataGenericItem_RelativePose(const DataGenericItem_RelativePose &copyObj);

    DataGenericItem_RelativePose(bool reference, const QJsonObject &position);

    //!
    //! \brief setDistance Assign the distance from the reference
    //! \param distance
    //!
    void setDistance(const double &distance){
        this->distance = distance;
    }

    //!
    //! \brief getDistance Return the distance from the reference
    //! \return distance
    //!
    double getDistance() const{
        return distance;
    }

    //!
    //! \brief setBearing Assign the bearing from the reference
    //! \param bearing
    //!
    void setBearing(const double &bearing){
        this->bearing = bearing;
    }

    //!
    //! \brief getBearing Return the bearing from the reference
    //! \return bearing
    //!
    double getBearing() const{
        return bearing;
    }

    //!
    //! \brief setAltitude Assign the altitude of the aircraft
    //! \param altitude
    //!
    void setAltitude(const double &altitude){
        this->altitude = altitude;
    }

    //!
    //! \brief getAltitude Return the altitude of the aircraft
    //! \return altitude
    //!
    double getAltitude() const{
        return altitude;
    }

    //!
    //! \brief setHeading Assign the heading offset from the reference
    //! \param heading
    //!
    void setHeading(const double &heading){
        this->heading = heading;
    }

    //!
    //! \brief getHeading Return the heading offset from the reference
    //! \return heading
    //!
    double getHeading() const{
        return heading;
    }

    //!
    //! \brief setReference Assign whether this aircraft is the reference
    //! \param isReference
    //!
    void setReference(const bool &isReference){
        this->isReference = isReference;
    }

    //!
    //! \brief getReference Return whether this aircraft is the reference
    //! \return isReference
    //!
    bool getReference() const{
        return isReference;
    }

public:
    void operator = (const DataGenericItem_RelativePose &rhs)
    {
        this->altitude = rhs.altitude;
        this->distance = rhs.distance;
        this->bearing = rhs.bearing;
        this->heading = rhs.heading;
        this->isReference = rhs.isReference;
    }

    bool operator == (const DataGenericItem_RelativePose &rhs) const {
        if(this->altitude != rhs.altitude){
            return false;
        }
        if(this->distance != rhs.distance){
            return false;
        }
        if(this->bearing != rhs.bearing){
            return false;
        }
        if(this->heading != rhs.heading){
            return false;
        }
        if(this->isReference != rhs.isReference){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_RelativePose &rhs) const {
        return !(*this == rhs);
    }

protected:
    bool isReference; //if true, the following parameters should be ignored
    double  distance; //2D distance from the reference aircraft's position (m)
    double  bearing; //Compass angle of the vector from the reference aircraft to this position (rad)
    double  altitude; //ground altitude, NOT dependent on reference aircraft (m)
    double  heading; //Compass heading of this aircraft where the heading of the reference aircraft is 0 (rad)
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_RELATIVEPOSE_H
