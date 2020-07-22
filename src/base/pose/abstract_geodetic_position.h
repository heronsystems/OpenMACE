#ifndef ABSTRACT_GEODETIC_POSITION_H
#define ABSTRACT_GEODETIC_POSITION_H

#include <sstream>

#include <Eigen/Core>

#include "mace.h"

#include "abstract_position.h"
#include "position_interface.h"
#include "data/jsonconverter.h"

namespace mace {
namespace pose {

MACE_CLASS_FORWARD(Abstract_GeodeticPosition);
MACE_CLASS_FORWARD(GeodeticPosition_2D);
MACE_CLASS_FORWARD(GeodeticPosition_3D);

class Abstract_GeodeticPosition : public JSONConverter, public Position, public PositionInterface<Abstract_GeodeticPosition>
{
public:
    Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const std::string &posName = "Position Object");

    Abstract_GeodeticPosition(const Abstract_GeodeticPosition &copy);

    ~Abstract_GeodeticPosition() override = default;

    virtual Abstract_GeodeticPosition* getGeodeticClone() const = 0;

    virtual void getGeodeticClone(Abstract_GeodeticPosition** state) const = 0;

    bool areEquivalentGeodeticFrames(const Abstract_GeodeticPosition &obj) const;

    /** Interface imposed via Position*/
public:
    PositionTypes getPositionalType() const
    {
        return PositionTypes::TRANSLATIONAL;
    }

    /** End of interface imposed via Position */

public:

//    virtual Eigen::VectorXd getDataVector() const = 0; //This function remains virtual from the base class of position since this class is Abstract
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;
    //!
    //! \brief setLatitude
    //! \param latitude
    //!
    virtual void setLatitude(const double &latitude) = 0;

    //!
    //! \brief setLongitude
    //! \param longitude
    //!
    virtual void setLongitude(const double &longitude) = 0;

    //!
    //! \brief getLatitude
    //! \return
    //!
    virtual double getLatitude() const = 0;

    //!
    //! \brief getLongitude
    //! \return
    //!
    virtual double getLongitude() const = 0;

    //!
    //! \brief deltaLatitude
    //! \param that
    //! \return
    //!
    double deltaLatitude(const Abstract_GeodeticPosition* that) const
    {
        return this->getLatitude() - that->getLatitude();
    }

    //!
    //! \brief deltaLongitude
    //! \param that
    //! \return
    //!
    double deltaLongitude(const Abstract_GeodeticPosition* that) const
    {
        return this->getLongitude() - that->getLongitude();
    }

public:
    void applyTransformation(const Eigen::Transform<double,2,Eigen::Affine> &t) override
    {
        UNUSED(t);
        //a rotation and translation in the geodetic frame does not make a whole lot of sense, therefore ignoring
    }

    void applyTransformation(const Eigen::Transform<double,3,Eigen::Affine> &t) override
    {
        UNUSED(t);
        //a rotation and translation in the geodetic frame does not make a whole lot of sense, therefore ignoring
    }

public:
    CoordinateSystemTypes getCoordinateSystemType() const override;

    void setCoordinateFrame(const GeodeticFrameTypes &explicitFrame);

    GeodeticFrameTypes getGeodeticCoordinateFrame() const;

    CoordinateFrameTypes getExplicitCoordinateFrame() const override;

    /** Assignment Operators */
public:
    Abstract_GeodeticPosition& operator = (const Abstract_GeodeticPosition &rhs)
    {
        Position::operator =(rhs);
        this->geodeticFrameType = rhs.geodeticFrameType;
        return *this;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Abstract_GeodeticPosition &rhs) const
    {
        if(!Position::operator ==(rhs))
            return false;

        if(this->geodeticFrameType != rhs.geodeticFrameType){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Abstract_GeodeticPosition &rhs) const {
        return !(*this == rhs);
    }

protected:
    GeodeticFrameTypes geodeticFrameType = GeodeticFrameTypes::CF_GLOBAL_UNKNOWN;
};

} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_CARTESIAN_POSITION_H
