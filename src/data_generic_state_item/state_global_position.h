#ifndef STATE_GLOBAL_POSITION_H
#define STATE_GLOBAL_POSITION_H

#include <iostream>
#include <cmath>

#include <Eigen/Dense>

#include "mace.h"
#include "common/common.h"
#include "data/coordinate_frame.h"
#include "abstract_3d_position.h"
#include "base_3d_position.h"

#include "base/pose/geodetic_position_3D.h"

namespace DataState {
//!
//! \brief The StateGlobalPosition class
//!
class StateGlobalPosition : public Abstract3DPosition<StateGlobalPosition>, public Base3DPosition
{
public:

    //!
    //! \brief StateGlobalPosition
    //!
    StateGlobalPosition();

    StateGlobalPosition(const mace::pose::GeodeticPosition_3D &position);

    //!
    //! \brief StateGlobalPosition
    //! \param globalPosition
    //!
    StateGlobalPosition(const StateGlobalPosition &globalPosition);

    //!
    //! \brief StateGlobalPosition
    //! \param frame
    //!
    StateGlobalPosition(const Data::CoordinateFrameType &frame);

    //!
    //! \brief StateGlobalPosition
    //! \param latitude
    //! \param longitude
    //! \param altitude
    //!
    StateGlobalPosition(const float &latitude, const float &longitude, const float &altitude);

    //!
    //! \brief StateGlobalPosition
    //! \param frame
    //! \param latitude
    //! \param longitude
    //! \param altitude
    //!
    StateGlobalPosition(const Data::CoordinateFrameType &frame, const double &latitude, const double &longitude, const double &altitude);

    //!
    //! \brief StateGlobalPosition
    //! \param pos
    //!
    StateGlobalPosition(const mace_global_position_int_t &pos);


public:
    //!
    //! \brief setPosition
    //! \param latitude
    //! \param longitude
    //! \param altitude
    //!
    void setPosition(const double &latitude, const double &longitude, const double &altitude);

    //!
    //! \brief setLatitude
    //! \param value
    //!
    void setLatitude(const double &value);

    //!
    //! \brief setLongitude
    //! \param value
    //!
    void setLongitude(const double &value);

    //!
    //! \brief setAltitude
    //! \param value
    //!
    void setAltitude(const double &value);

    //!
    //! \brief getLatitude
    //! \return
    //!
    double getLatitude() const;

    //!
    //! \brief getLongitude
    //! \return
    //!
    double getLongitude() const;

    //!
    //! \brief getAltitude
    //! \return
    //!
    double getAltitude() const;

public:
    mace_global_position_int_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

//virtual functions derived from AbstractPosition
public:
    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const StateGlobalPosition &position) const;

    //!
    //! \brief finalBearing
    //! \param postion
    //! \return
    //!
    virtual double finalBearing(const StateGlobalPosition &postion) const;

    //!
    //! \brief initialBearing
    //! \param postion
    //! \return
    //!
    virtual double initialBearing(const StateGlobalPosition &postion) const;

    //!
    //! \brief bearingBetween
    //! \param position
    //! \return
    //!
    virtual double bearingBetween(const StateGlobalPosition &position) const;

    //!
    //! \brief NewPositionFromHeadingBearing
    //! \param distance
    //! \param bearing
    //! \param degreesFlag
    //! \return
    //!
    virtual StateGlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const;

    //!
    //! \brief translationTransformation2D
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation2D(const StateGlobalPosition &position, Eigen::Vector2f &transVec) const;

//virtual functions derived from Abstract3DPosition
public:
    //!
    //! \brief deltaAltitude
    //! \param position
    //! \return
    //!
    virtual double deltaAltitude(const StateGlobalPosition &position) const;

    //!
    //! \brief distanceBetween3D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween3D(const StateGlobalPosition &position) const;

    //!
    //! \brief translationTransformation3D
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation3D(const StateGlobalPosition &position, Eigen::Vector3f &transVec) const;

public:

    //!
    //! \brief operator =
    //! \param rhs
    //!
    void operator = (const StateGlobalPosition &rhs)
    {
        Base3DPosition::operator =(rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const StateGlobalPosition &rhs) {

        if(!Base3DPosition::operator ==(rhs)){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const StateGlobalPosition &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace DataState

#endif // STATE_GLOBAL_POSITION_H
