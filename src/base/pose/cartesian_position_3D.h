#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include "abstract_altitude.h"
#include "abstract_cartesian_position.h"
#include "../state_space/state.h"

using namespace mace::math;

namespace mace {
namespace pose {

class CartesianPosition_3D : public Abstract_CartesianPosition, public Abstract_Altitude, public state_space::State
{
public:

    CartesianPosition_3D();

    CartesianPosition_3D(const CartesianFrameTypes &frameType,
                        const double &x, const double &y,
                        const AltitudeReferenceTypes &altitudeType, const double &z,
                        const std::string &pointName = "Cartesian Point");

    CartesianPosition_3D(const double &x, const double &y, const double &z, const std::string &pointName = "Cartesian Point");

    CartesianPosition_3D(const CartesianPosition_3D &copy);

    CartesianPosition_3D(const CartesianPosition_2D &copy);


    ~CartesianPosition_3D() override = default;

    Abstract_CartesianPosition* getCartesianClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getCartesianClone(Abstract_CartesianPosition** state) const override
    {
        *state = new CartesianPosition_3D(*this);
    }

    bool areEquivalentFrames(const CartesianPosition_3D &obj) const;

public:
    Eigen::VectorXd getDataVector() const override
    {
        return this->data;
    }

    void updateFromDataVector(const Eigen::VectorXd &vec) override
    {
        long rows = vec.rows();
        if(rows >= 2)
        {
            this->setXPosition(vec(0));
            this->setYPosition(vec(1));
        }

        if(rows == 3)
            this->setAltitude(vec(2));
    }

    Eigen::Vector3d retrieveDataVector() const
    {
        return data;
    }


public:
    void updatePosition(const double &x, const double &y, const double &z)
    {
        this->setTranslationalPosition(x,y);
        this->setAltitude(z);
    }

    void setTranslationalPosition(const double &x, const double &y)
    {
        this->setXPosition(x);
        this->setYPosition(y);
    }
    void setXPosition(const double &x)
    {
        this->data(0) = x;
        this->validateDimension(IGNORE_X_DIMENSION);
    }

    void setYPosition(const double &y)
    {
        this->data(1) = y;
        this->validateDimension(IGNORE_Y_DIMENSION);
    }

    void setZPosition(const double &z)
    {
        this->data(2) = z;
        this->validateDimension(IGNORE_Z_DIMENSION);
    }

    double getXPosition() const
    {
        return this->data(0);
    }

    double getYPosition() const
    {
        return this->data(1);
    }

    double getZPosition() const
    {
        return this->data(2);
    }


public:
    bool hasXBeenSet() const;

    bool hasYBeenSet() const;

    bool hasZBeenSet() const;

    bool hasTranslationalComponentBeenSet() const;

public:
    double deltaX(const CartesianPosition_3D &that) const;
    double deltaY(const CartesianPosition_3D &that) const;
    double deltaZ(const CartesianPosition_3D &that) const;

public:
    void applyTransformation(const Eigen::Transform<double,2,Eigen::Affine> &t) override;

    void applyTransformation(const Eigen::Transform<double,3,Eigen::Affine> &t) override;

public:
    /** Interface imposed via AltitudeInterface */

    //!
    //! \brief setAltitude
    //! \param altitude
    //!
    void setAltitude(const double &altitude) override;

    //!
    //! \brief getAltitude
    //! \return
    //!
    double getAltitude() const override;

    //!
    //! \brief elevationAngleFromOrigin
    //! \return
    //!
    double elevationAngleFromOrigin() const override;

    //!
    //! \brief elevationAngleTo
    //! \param position
    //! \return
    //!
    double elevationAngleTo(const Abstract_CartesianPosition* position) const;



    /** Interface imposed via state_space::State */
public:
    State* getStateClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getStateClone(State** state) const override
    {
        *state = new CartesianPosition_3D(*this);
    }


    /** Interface imposed via Abstract_CartesianPosition */

public:
    Position* getPositionalClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getPositionalClone(Position** state) const override
    {
        *state = new CartesianPosition_3D(*this);
    }

public:
    void updateQJSONObject(QJsonObject &obj) const override;

public:
    //!
    //! \brief distanceFromOrigin
    //! \return
    //!
    double distanceFromOrigin() const override;

    //!
    //! \brief translationalDistanceFromOrigin
    //! \return
    //!
    double translationalDistanceFromOrigin() const override;

    //!
    //! \brief polarBearingFromOrigin
    //! \return
    //!
    double polarBearingFromOrigin() const override;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    double distanceBetween2D(const Abstract_CartesianPosition* pos) const override;

    double distanceBetween3D(const Abstract_CartesianPosition* pos) const;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const Abstract_CartesianPosition* pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const Abstract_CartesianPosition* pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const Abstract_CartesianPosition* pos) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    void newPositionFromPolar(Abstract_CartesianPosition* newObject, const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    void newPositionFromCompass(Abstract_CartesianPosition *newObject, const double &distance, const double &bearing) const override;

    void applyPositionalShiftFromPolar(const double &distance, const double &bearing) override;

    void applyPositionalShiftFromPolar(const double &distance, const double &bearing, const double &elevation);

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing) override;

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing, const double &elevation);

public:
    mavlink_local_position_ned_t getMACE_CartesianPositionInt() const;

    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const override;

    /** Assignment Operators */
public:
    CartesianPosition_3D& operator = (const CartesianPosition_3D &rhs)
    {
        Abstract_CartesianPosition::operator =(rhs);
        Abstract_Altitude::operator =(rhs);
        this->data = rhs.data;
        return *this;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const CartesianPosition_3D &rhs) const
    {
        if(!Abstract_CartesianPosition::operator ==(rhs))
            return false;
        if(!Abstract_Altitude::operator ==(rhs))
            return false;
        if(!this->data.isApprox(rhs.data, std::numeric_limits<double>::epsilon()))
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const CartesianPosition_3D &rhs) const {
        return !(*this == rhs);
    }
public:
    CartesianPosition_3D& operator+= (const CartesianPosition_3D &rhs)
    {

        if(this->areEquivalentFrames(rhs))
        {
            this->data += rhs.data;
            return *this;
        }
        else
            throw std::logic_error("Tried to perform a + operation between 2DCartesians of differnet coordinate frames.");
    }

public:
    std::string printPositionalInfo() const override
    {
        std::stringstream stream;
        stream.precision(6);
        stream << std::fixed << "X:" << this->getXPosition() << ", Y:"<< this->getYPosition() <<", Z:"<< this->getZPosition()<<".";
        return stream.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const CartesianPosition_3D& t)
    {
        std::stringstream newStream;
        t.printPositionLog(newStream);
        os << newStream.str();
        return os;
    }

public:
    
    Eigen::Vector3d data;
};


/*!
 * @brief Overloaded plus operator for Vectors.
 */
BASESHARED_EXPORT CartesianPosition_3D operator +(const CartesianPosition_3D &a, const CartesianPosition_3D &b );

/*!
 * @brief Overloaded minus operator for Vectors.
 */
BASESHARED_EXPORT CartesianPosition_3D operator -(const CartesianPosition_3D &a, const CartesianPosition_3D &b );

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
