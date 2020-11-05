#include "cartesian_position_2D.h"
#include "cartesian_position_3D.h"

namespace mace{
namespace pose{


CartesianPosition_3D::CartesianPosition_3D():
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, "Cartesian Point"), Abstract_Altitude(AltitudeReferenceTypes::REF_ALT_RELATIVE), State(), data(0.0,0.0,0.0)
{
    this->dimension = 3;
    this->setDimensionMask(ignoreAllPositions);
}

CartesianPosition_3D::CartesianPosition_3D(const CartesianFrameTypes &frameType,
                    const double &x, const double &y,
                    const AltitudeReferenceTypes &altitudeType, const double &z,
                    const std::string &pointName):
    Abstract_CartesianPosition(frameType, pointName), Abstract_Altitude(altitudeType), State(), data(0.0,0.0,0.0)

{
    this->dimension = 3;
    this->setXPosition(x); this->setYPosition(y); this->setZPosition(z);
}

CartesianPosition_3D::CartesianPosition_3D(const double &x, const double &y, const double &z, const std::string &pointName):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), Abstract_Altitude(AltitudeReferenceTypes::REF_ALT_RELATIVE), State(), data(0.0,0.0,0.0)
{
    this->dimension = 3;
    this->updatePosition(x,y,z);
}

CartesianPosition_3D::CartesianPosition_3D(const CartesianPosition_3D &copy):
    Abstract_CartesianPosition(copy), Abstract_Altitude(copy), state_space::State(copy), data(copy.data)
{
    this->dimension = 3;
    this->updatePosition(copy.getXPosition(), copy.getYPosition(), copy.getZPosition());
}

CartesianPosition_3D::CartesianPosition_3D(const CartesianPosition_2D &copy):
    Abstract_CartesianPosition(copy), Abstract_Altitude(), state_space::State(copy), data(copy.data(0), copy.data(1), 0.0)
{
    this->dimension = 3;
}

bool CartesianPosition_3D::hasXBeenSet() const
{
    if((this->dimensionMask&IGNORE_X_DIMENSION) == 0)
        return true;
    return false;
}

bool CartesianPosition_3D::hasYBeenSet() const
{
    if((this->dimensionMask&IGNORE_Y_DIMENSION) == 0)
        return true;
    return false;
}

bool CartesianPosition_3D::hasZBeenSet() const
{
    if((this->dimensionMask&IGNORE_Z_DIMENSION) == 0)
        return true;
    return false;
}

bool CartesianPosition_3D::hasTranslationalComponentBeenSet() const
{
    return hasXBeenSet() || hasYBeenSet();
}

bool CartesianPosition_3D::areEquivalentFrames(const CartesianPosition_3D &obj) const
{
    return this->areEquivalentCartesianFrames(obj) && this->areEquivalentAltitudeFrames(&obj);
}

void CartesianPosition_3D::updateQJSONObject(QJsonObject &obj) const
{
    UNUSED(obj);
    //PAT: Would this situation have ever occured
}

double CartesianPosition_3D::deltaX(const CartesianPosition_3D &that) const
{
    return this->data(0) - that.data(0);
}

double CartesianPosition_3D::deltaY(const CartesianPosition_3D &that) const
{
    return this->data(1) - that.data(1);
}

double CartesianPosition_3D::deltaZ(const CartesianPosition_3D &that) const
{
    return this->data(2) - that.data(2);
}

//!
//! \brief setAltitude
//! \param altitude
//!
void CartesianPosition_3D::setAltitude(const double &altitude)
{
    this->setZPosition(altitude);
}

//!
//! \brief getAltitude
//! \return
//!
double CartesianPosition_3D::getAltitude() const
{
    return this->data(2);
}

void CartesianPosition_3D::applyTransformation(const Eigen::Transform<double, 2, Eigen::Affine> &t)
{
    //since this is only a 2D object we have to reconstruct a new transform
    Eigen::Transform<double, 3, Eigen::Affine> currentTransform;
    currentTransform.translation() = Eigen::Vector3d(t.translation().x(), t.translation().y(), 0);
    currentTransform.linear() = Eigen::Matrix3d::Identity();
    currentTransform.linear().block(0,0,2,2) = t.rotation();
    this->data = currentTransform.linear() * data + currentTransform.translation();
}

void CartesianPosition_3D::applyTransformation(const Eigen::Transform<double, 3, Eigen::Affine> &t)
{
    this->data = t.linear() * data + t.translation();
}

double CartesianPosition_3D::distanceFromOrigin() const
{
    return this->data.norm();
}

double CartesianPosition_3D::translationalDistanceFromOrigin() const
{
    CartesianPosition_3D origin;
    double distanceXY = this->distanceBetween2D(&origin);
    return distanceXY;
}

double CartesianPosition_3D::polarBearingFromOrigin() const
{
    return atan2(this->getYPosition(),this->getXPosition());
}

double CartesianPosition_3D::elevationAngleFromOrigin() const
{
    CartesianPosition_3D origin;
    double distanceXY = this->distanceBetween2D(&origin);
    return atan2(this->getZPosition(),distanceXY); //we should check here for the discontinuity if they are both 0
}

double CartesianPosition_3D::elevationAngleTo(const Abstract_CartesianPosition* pos) const
{
    double elevationAngle = 0.0;

    if(pos->is3D())
    {
        const CartesianPosition_3D* tmpPos = pos->positionAs<CartesianPosition_3D>();
        double distance2D = distanceBetween2D(pos);
        double distanceAlt = deltaAltitude(tmpPos);

        if(fabs(distance2D) > std::numeric_limits<double>::epsilon() || fabs(distanceAlt) > std::numeric_limits<double>::epsilon())
                elevationAngle = atan2(deltaAltitude(tmpPos),distance2D);
    }
    return elevationAngle;
}

double CartesianPosition_3D::distanceBetween2D(const Abstract_CartesianPosition* pos) const
{
    double distance = 0.0;
    if(pos->isGreaterThan2D())
    {
        const CartesianPosition_3D* tmpPos = pos->positionAs<CartesianPosition_3D>();
        Eigen::Vector3d delta = this->data - tmpPos->data;
        distance = delta.head(2).norm();
    }
    else if(pos->isGreaterThan1D())
    {
        const CartesianPosition_2D* tmpPos = pos->positionAs<CartesianPosition_2D>();
        Eigen::Vector2d delta = this->data.head(2) - tmpPos->data;
        distance = delta.norm();
    }
    return distance;
}

double CartesianPosition_3D::distanceBetween3D(const Abstract_CartesianPosition* pos) const
{
    double distance = 0.0;
    if(pos->is3D())
    {
        const CartesianPosition_3D* tmpPos = pos->positionAs<CartesianPosition_3D>();
        Eigen::Vector3d delta = this->data - tmpPos->data;
        distance = delta.norm();
    }
    else
    {
        distance = this->distanceBetween2D(pos);
    }

    return distance;
}

// ** DEPRECATED ** //
double CartesianPosition_3D::distanceTo(const Abstract_CartesianPosition* pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::polarBearingTo(const Abstract_CartesianPosition* pos) const
{
    double polarBearing = 0.0;

    if(pos->isGreaterThan1D())
    {
        const CartesianPosition_2D* tmpPos = pos->positionAs<CartesianPosition_2D>();
        double deltaY = tmpPos->getYPosition() - this->getYPosition();
        double deltaX = tmpPos->getXPosition() - this->getXPosition();
        polarBearing = atan2(deltaY,deltaX);
    }

    return polarBearing;
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::compassBearingTo(const Abstract_CartesianPosition* pos) const
{
    return math::polarToCompassBearing(polarBearingTo(pos));
}

//!
//! \brief CartesianPosition_2D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
void CartesianPosition_3D::newPositionFromPolar(Abstract_CartesianPosition *newObject, const double &distance, const double &bearing) const
{
    if(newObject->isGreaterThan1D())
    {
        CartesianPosition_3D* tmpPos = newObject->positionAs<CartesianPosition_3D>();
        tmpPos->setXPosition(this->getXPosition() + std::cos(bearing) * distance);
        tmpPos->setYPosition(this->getYPosition() + std::sin(bearing) * distance);
    }
}


//!
//! \brief CartesianPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
void CartesianPosition_3D::newPositionFromCompass(Abstract_CartesianPosition* newObject, const double &distance, const double &bearing) const
{
    return newPositionFromPolar(newObject, distance,compassToPolarBearing(bearing));
}

void CartesianPosition_3D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    double changeX = distance * std::cos(bearing);
    double changeY = distance * std::sin(bearing);
    this->setXPosition(getXPosition() + changeX);
    this->setYPosition(getYPosition() + changeY);
}

void CartesianPosition_3D::applyPositionalShiftFromPolar(const double &distance, const double &bearing, const double &elevation)
{
    double changeZ = distance * std::sin(elevation);
    this->setZPosition(changeZ + this->getZPosition());

    double changeXY = distance * std::cos(elevation);
    applyPositionalShiftFromPolar(changeXY,bearing);
}

void CartesianPosition_3D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    double polarBearing = compassToPolarBearing(bearing);
    applyPositionalShiftFromPolar(distance,polarBearing);
}

void CartesianPosition_3D::applyPositionalShiftFromCompass(const double &distance, const double &bearing, const double &elevation)
{
    applyPositionalShiftFromPolar(distance,compassToPolarBearing(bearing),elevation);
}

mace_local_position_ned_t CartesianPosition_3D::getMACE_CartesianPositionInt() const
{
    mace_local_position_ned_t posObj;
    posObj.x = static_cast<int32_t>((this->getXPosition() * pow(10,7)));
    posObj.y = static_cast<int32_t>((this->getYPosition() * pow(10,7)));
    posObj.z = static_cast<int32_t>((this->getZPosition() * pow(10,7)));
    posObj.time_boot_ms = 0;
    return posObj;
}

mace_message_t CartesianPosition_3D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_local_position_ned_t positionObj = getMACE_CartesianPositionInt();
    mace_msg_local_position_ned_encode_chan(systemID,compID,chan,&msg,&positionObj);
    return msg;
}

CartesianPosition_3D operator+ (const CartesianPosition_3D &lhs, const CartesianPosition_3D &rhs)
{

    if(lhs.areEquivalentFrames(rhs))
    {
        CartesianPosition_3D newPoint(lhs);
        Eigen::Vector3d result = lhs.data + rhs.data;
        newPoint.updatePosition(result(0), result(1), result(2));
        return newPoint;
    }
    else
        throw std::logic_error("Tried to perform a + operation between 3DCartesians of differnet coordinate frames.");
}

CartesianPosition_3D operator- (const CartesianPosition_3D &lhs, const CartesianPosition_3D &rhs)
{

    if(lhs.areEquivalentFrames(rhs))
    {
        CartesianPosition_3D newPoint(lhs);
        Eigen::Vector3d result = lhs.data - rhs.data;
        newPoint.updatePosition(result(0), result(1), result(2));
        return newPoint;
    }
    else
        throw std::logic_error("Tried to perform a - operation between 3DCartesians of differnet coordinate frames.");

}


} //end of namespace pose
} //end of namespace mace
