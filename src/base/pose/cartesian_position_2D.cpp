#include "cartesian_position_2D.h"
#include "cartesian_position_3D.h"

namespace mace{
namespace pose{

CartesianPosition_2D::CartesianPosition_2D(const CartesianFrameTypes &frameType,
                    const double &x, const double &y,
                    const std::string &pointName):
    Abstract_CartesianPosition(frameType, x, y, pointName), State()
{
    this->dimension = 2;
}

CartesianPosition_2D::CartesianPosition_2D(const std::string &pointName,
                    const double &x, const double &y):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, x, y, pointName), State()
{
    this->dimension = 2;
}

CartesianPosition_2D::CartesianPosition_2D(const double &x, const double &y):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, x, y, "Cartesian Point"), State()
{
    this->dimension = 2;
}

CartesianPosition_2D::CartesianPosition_2D(const CartesianPosition_2D &copy):
    Abstract_CartesianPosition(copy), state_space::State(copy)
{

}

CartesianPosition_2D::CartesianPosition_2D(const CartesianPosition_3D &copy):
    Abstract_CartesianPosition(copy), state_space::State(copy)
{
    this->dimension = 2;
}

bool CartesianPosition_2D::areEquivalentFrames(const CartesianPosition_2D &obj) const
{
    return this->getCartesianFrameType() == obj.getCartesianFrameType();
}

double CartesianPosition_2D::deltaX(const CartesianPosition_2D &that) const
{
    return this->getXPosition() - that.getXPosition();
}

double CartesianPosition_2D::deltaY(const CartesianPosition_2D &that) const
{
    return this->getYPosition() - that.getYPosition();
}

double CartesianPosition_2D::distanceFromOrigin() const
{
    return sqrt(pow(this->getXPosition(),2) + pow(this->getYPosition(),2));
}

double CartesianPosition_2D::polarBearingFromOrigin() const
{
    return atan2(this->getYPosition(),this->getXPosition());
}

double CartesianPosition_2D::distanceBetween2D(const Abstract_CartesianPosition* pos) const
{
    double distance = 0.0;

    if(pos->isGreaterThan1D())
    {
        const CartesianPosition_2D* tmpPos = pos->positionAs<CartesianPosition_2D>();
        double deltaX = this->getX() - tmpPos->getX();
        double deltaY = this->getY() - tmpPos->getY();
        distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
    }
    return distance;
}

double CartesianPosition_2D::distanceBetween2D(const CartesianPosition_2D &pos) const
{
    return distanceBetween2D(&pos);
}

double CartesianPosition_2D::distanceTo(const Abstract_CartesianPosition* pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief CartesianPosition_2D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_2D::polarBearingTo(const Abstract_CartesianPosition* pos) const
{
    double polarBearing = 0.0;
    if(pos->isGreaterThan1D())
    {
        const CartesianPosition_2D* tmpPos = pos->positionAs<CartesianPosition_2D>();
        polarBearing = atan2(deltaY(*tmpPos),deltaX(*tmpPos));
    }

    return polarBearing;
}

//!
//! \brief CartesianPosition_2D::compassBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_2D::compassBearingTo(const Abstract_CartesianPosition* pos) const
{
    return math::polarToCompassBearing(polarBearingTo(pos));
}

//!
//! \brief CartesianPosition_2D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
void CartesianPosition_2D::newPositionFromPolar(Abstract_CartesianPosition *newObject, const double &distance, const double &bearing) const
{
    if(newObject->isGreaterThan1D())
    {
        CartesianPosition_2D* tmpPos = newObject->positionAs<CartesianPosition_2D>();
        tmpPos->setXPosition(this->getXPosition() + cos(bearing) * distance);
        tmpPos->setYPosition(this->getYPosition() + sin(bearing) * distance);
    }
}

//!
//! \brief CartesianPosition_2D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
void CartesianPosition_2D::newPositionFromCompass(Abstract_CartesianPosition* newObject, const double &distance, const double &bearing) const
{
    return newPositionFromPolar(newObject, distance, math::compassToPolarBearing(bearing));
}

void CartesianPosition_2D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    double changeX = distance * cos(bearing);
    double changeY = distance * sin(bearing);
    this->setXPosition(getXPosition() + changeX);
    this->setYPosition(getYPosition() + changeY);
}

void CartesianPosition_2D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    double polarBearing = math::compassToPolarBearing(bearing);
    applyPositionalShiftFromPolar(distance,polarBearing);
}

void CartesianPosition_2D::normalize()
{
    double magnitude = sqrt(pow(getXPosition(),2) + pow(getYPosition(),2));
    this->setXPosition(getXPosition()/magnitude);
    this->setYPosition(getYPosition()/magnitude);
}

void CartesianPosition_2D::scale(const double &value)
{
    this->setXPosition(getXPosition()*value);
    this->setYPosition(getYPosition()*value);
}

std::ostream& operator<<(std::ostream& os, const CartesianPosition_2D& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Cartesian Position 2D: "<<
              t.getXPosition() << ", "<< t.getYPosition() << ".";
    os << stream.str();

    return os;
}


} //end of namespace pose
} //end of namespace mace
