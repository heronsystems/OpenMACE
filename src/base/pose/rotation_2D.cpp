#include "rotation_2D.h"
#include "rotation_3D.h"

namespace mace {
namespace pose {

Rotation_2D::Rotation_2D(const std::string &name):
    AbstractRotation(1,name), Eigen::Rotation2D<double>()
{
    this->dimensionMask = ignoreAllPositions;
}

Rotation_2D::Rotation_2D(const Rotation_2D &copy):
    AbstractRotation(copy), Eigen::Rotation2D<double> (copy.angle()), dimensionMask(copy.dimensionMask)
{

}

Rotation_2D::Rotation_2D(const Rotation_3D &copy):
    AbstractRotation(copy), Eigen::Rotation2D<double>(copy.getYaw())
{

}

Rotation_2D::Rotation_2D(const double &angle):
    AbstractRotation(1)
{
    this->setPhi(angle);
}

bool Rotation_2D::isYawDimensionSet() const
{
    if((this->dimensionMask&YAW_DIMENSION_VALID) == 0)
        return true;
    return false;
}

void Rotation_2D::setPhi(const double &angle)
{
    this->angle() = angle;
    this->setDimensionMask(YAW_DIMENSION_VALID);
}

double Rotation_2D::getPhi() const
{
    return this->angle();
}

void Rotation_2D::setQuaternion(const Eigen::Quaterniond &rot)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(rot);
    this->setPhi(currentRotation.alpha());
}

Eigen::Quaterniond Rotation_2D::getQuaternion() const
{
    Eigen::AngleAxisd rotation_vector (this->getPhi(), Eigen::Vector3d(0, 0, 1));
    return Eigen::Quaterniond(rotation_vector);
}

mavlink_attitude_quaternion_t Rotation_2D::getMACEQuaternion() const
{
    mavlink_attitude_quaternion_t quat;

    return quat;
}

mavlink_attitude_t Rotation_2D::getMACEEuler() const
{
    mavlink_attitude_t euler;
    euler.yaw = static_cast<float>(this->getPhi());
    return euler;
}

mavlink_message_t Rotation_2D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_attitude_t attitude = getMACEEuler();
    mavlink_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}


} //end of namespace pose
} //end of namespace mace
