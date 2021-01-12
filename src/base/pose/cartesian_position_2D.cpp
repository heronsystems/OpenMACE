#include "cartesian_position_2D.h"
#include "cartesian_position_3D.h"

namespace mace{
namespace pose{

CartesianPosition_2D::CartesianPosition_2D():
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_UNKNOWN, ""), State(), data(0,0)
{
    this->dimension = 2;
    this->setDimensionMask(ignoreAllPositions);
}

CartesianPosition_2D::CartesianPosition_2D(const CartesianFrameTypes &frameType,
                    const double &x, const double &y,
                    const std::string &pointName):
    Abstract_CartesianPosition(frameType, pointName), State(), data(0.0,0.0)
{
    this->dimension = 2;
    this->setXPosition(x); this->setYPosition(y);
}

CartesianPosition_2D::CartesianPosition_2D(const std::string &pointName,
                    const double &x, const double &y):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), State(), data(0.0,0.0)
{
    this->dimension = 2;
    this->setXPosition(x); this->setYPosition(y);
}

CartesianPosition_2D::CartesianPosition_2D(const double &x, const double &y):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, "Cartesian Point"), State(), data(0.0,0.0)
{
    this->dimension = 2;
    this->updatePosition(x, y);
}

CartesianPosition_2D::CartesianPosition_2D(const CartesianPosition_2D &copy):
    Abstract_CartesianPosition(copy), state_space::State(copy), data(copy.data)
{
    this->dimension = 2;
}

CartesianPosition_2D::CartesianPosition_2D(const CartesianPosition_3D &copy):
    Abstract_CartesianPosition(copy), state_space::State(copy), data(0.0,0.0)
{
    this->dimension = 2;

    data(0) = copy.data(0);
    data(1) = copy.data(1);
}

bool CartesianPosition_2D::hasXBeenSet() const
{
    if((this->dimensionMask&IGNORE_X_DIMENSION) == 0)
        return true;
    return false;
}

bool CartesianPosition_2D::hasYBeenSet() const
{
    if((this->dimensionMask&IGNORE_Y_DIMENSION) == 0)
        return true;
    return false;
}

bool CartesianPosition_2D::areEquivalentFrames(const CartesianPosition_2D &obj) const
{
    return this->getCartesianCoordinateFrame() == obj.getCartesianCoordinateFrame();
}

void CartesianPosition_2D::updateQJSONObject(QJsonObject &obj) const
{
    UNUSED(obj);
    //PAT: Would this situation have ever occured
}

double CartesianPosition_2D::deltaX(const CartesianPosition_2D &that) const
{
    return this->getXPosition() - that.getXPosition();
}

double CartesianPosition_2D::deltaY(const CartesianPosition_2D &that) const
{
    return this->getYPosition() - that.getYPosition();
}

void CartesianPosition_2D::applyTransformation(const Eigen::Transform<double, 2, Eigen::Affine> &t)
{
    this->data = t.linear() * data + t.translation();
}

void CartesianPosition_2D::applyTransformation(const Eigen::Transform<double, 3, Eigen::Affine> &t)
{
    //since this is only a 2D object we have to reconstruct
    Eigen::Transform<double, 2, Eigen::Affine> currentTransform;
    currentTransform.translation() = Eigen::Vector2d(t.translation().x(), t.translation().y());
    currentTransform.linear() = Eigen::Matrix2d(t.rotation().block(0,0,2,2));
    this->data = currentTransform.linear() * data + currentTransform.translation();
}

double CartesianPosition_2D::distanceFromOrigin() const
{
    return data.norm();
}

double CartesianPosition_2D::translationalDistanceFromOrigin() const
{
    return distanceFromOrigin();
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
        Eigen::Vector2d delta = this->data - tmpPos->data;
        distance = delta.norm();
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
        Eigen::VectorXd posVector = pos->getDataVector();
        double deltaY = posVector(1) - this->getYPosition();
        double deltaX = posVector(0) - this->getXPosition();
        polarBearing = atan2(deltaY,deltaX);
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
        tmpPos->setXPosition(this->getXPosition() + std::cos(bearing) * distance);
        tmpPos->setYPosition(this->getYPosition() + std::sin(bearing) * distance);
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
    newPositionFromPolar(newObject, distance, math::compassToPolarBearing(bearing));
}

void CartesianPosition_2D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    double changeX = distance * std::cos(bearing);
    double changeY = distance * std::sin(bearing);
    this->setXPosition(getXPosition() + changeX);
    this->setYPosition(getYPosition() + changeY);
}

void CartesianPosition_2D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    double polarBearing = math::compassToPolarBearing(bearing);
    applyPositionalShiftFromPolar(distance,polarBearing);
}

mavlink_local_position_ned_t CartesianPosition_2D::getMACE_CartesianPositionInt() const
{
    mavlink_local_position_ned_t posObj;
    posObj.x = static_cast<int32_t>((this->getXPosition() * pow(10,7)));
    posObj.y = static_cast<int32_t>((this->getYPosition() * pow(10,7)));
    posObj.z = 0.0;
    posObj.time_boot_ms = 0;
    return posObj;
}

mavlink_message_t CartesianPosition_2D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_local_position_ned_t positionObj = getMACE_CartesianPositionInt();
    mavlink_msg_local_position_ned_encode_chan(systemID,compID,chan,&msg,&positionObj);
    return msg;
}

CartesianPosition_2D operator+ (const CartesianPosition_2D &lhs, const CartesianPosition_2D &rhs)
{

    if(lhs.areEquivalentFrames(rhs))
    {
        CartesianPosition_2D newPoint(lhs);
        Eigen::Vector2d result = lhs.data + rhs.data;
        newPoint.updatePosition(result(0), result(1));
        return newPoint;
    }
    else
        throw std::logic_error("Tried to perform a + operation between 2DCartesians of differnet coordinate frames.");
}


CartesianPosition_2D operator- (const CartesianPosition_2D &lhs, const CartesianPosition_2D &rhs)
{

    if(lhs.areEquivalentFrames(rhs))
    {
        CartesianPosition_2D newPoint(lhs);
        Eigen::Vector2d result = lhs.data - rhs.data;
        newPoint.updatePosition(result(0), result(1));
        return newPoint;
    }
    else
        throw std::logic_error("Tried to perform a - operation between 2DCartesians of differnet coordinate frames.");

}


} //end of namespace pose
} //end of namespace mace
