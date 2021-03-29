#ifndef STATE_LOCAL_POSITION_H
#define STATE_LOCAL_POSITION_H

#include <iostream>

#include "mace.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "abstract_3d_position.h"
#include "base_3d_position.h"

#include "base/pose/cartesian_position_3D.h"

namespace DataState {

class StateLocalPosition : public Abstract3DPosition<StateLocalPosition>, public Base3DPosition
{
public:
    StateLocalPosition();

    StateLocalPosition(const mace::pose::CartesianPosition_3D &position);

    StateLocalPosition(const StateLocalPosition &localPosition);

    StateLocalPosition(const Data::CoordinateFrameType &frame);

    StateLocalPosition(const double &x, const double &y, const double &z);

    StateLocalPosition(const Data::CoordinateFrameType &frame, const double &x, const double &y, const double &z);

    StateLocalPosition(const mace_local_position_ned_t &pos);

public:
    void setPosition(const double &posX, const double &posY, const double &posZ);
    void setPositionX(const double &value);
    void setPositionY(const double &value);
    void setPositionZ(const double &value);

    double getPositionX() const;
    double getPositionY() const;
    double getPositionZ() const;

    double bearingDegreesFromOrigin() const;
    double distanceFromOrigin() const;

    mace_local_position_ned_t getMACECommsObject();

    //virtual functions derived from AbstractPosition
    public:
        //!
        //! \brief distanceBetween2D
        //! \param position
        //! \return
        //!
        virtual double distanceBetween2D(const StateLocalPosition &position) const;

        //!
        //! \brief finalBearing
        //! \param postion
        //! \return
        //!
        virtual double finalBearing(const StateLocalPosition &postion) const;

        //!
        //! \brief initialBearing
        //! \param postion
        //! \return
        //!
        virtual double initialBearing(const StateLocalPosition &postion) const;

        //!
        //! \brief bearingBetween
        //! \param position
        //! \return
        //!
        virtual double bearingBetween(const StateLocalPosition &position) const;

        //!
        //! \brief NewPositionFromHeadingBearing
        //! \param distance
        //! \param bearing
        //! \param degreesFlag
        //! \return
        //!
        virtual StateLocalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const;

        //!
        //! \brief translationTransformation2D
        //! \param position
        //! \param transVec
        //!
        virtual void translationTransformation2D(const StateLocalPosition &position, Eigen::Vector2f &transVec) const;

    //virtual functions derived from Abstract3DPosition
    public:
        //!
        //! \brief deltaAltitude
        //! \param position
        //! \return
        //!
        virtual double deltaAltitude(const StateLocalPosition &position) const;

        //!
        //! \brief distanceBetween3D
        //! \param position
        //! \return
        //!
        virtual double distanceBetween3D(const StateLocalPosition &position) const;

        //!
        //! \brief translationTransformation3D
        //! \param position
        //! \param transVec
        //!
        virtual void translationTransformation3D(const StateLocalPosition &position, Eigen::Vector3f &transVec) const;

public:
    bool essentiallyEquivalent_Percentage(const StateLocalPosition &rhs, const double &percentage);
    bool essentiallyEquivalent_Distance(const StateLocalPosition &rhs, const double &distance);

public:
    void operator = (const StateLocalPosition &rhs)
    {
        Base3DPosition::operator =(rhs);
    }

    bool operator == (const StateLocalPosition &rhs) {
        if(!Base3DPosition::operator ==(rhs)){
            return false;
        }
        return true;
    }

    bool operator != (const StateLocalPosition &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace DataState

#endif // STATE_LOCAL_POSITION_H
