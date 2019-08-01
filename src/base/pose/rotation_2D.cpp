#include "rotation_2D.h"
#include "rotation_3D.h"

namespace mace {
namespace pose {

Rotation_2D::Rotation_2D(const std::string &name):
    AbstractRotation(1,name), Eigen::Rotation2D<double>()
{
}

Rotation_2D::Rotation_2D(const Rotation_2D &copy):
    AbstractRotation(copy), Eigen::Rotation2D<double> (copy.angle())
{

}

Rotation_2D::Rotation_2D(const Rotation_3D &copy):
    AbstractRotation(copy), Eigen::Rotation2D<double>(copy.getYaw())
{
}

Rotation_2D::Rotation_2D(const double &angle):
    AbstractRotation(1), Eigen::Rotation2D<double>(angle)
{

}

void Rotation_2D::setPhi(const double &angle)
{
    this->angle() = angle;
}

double Rotation_2D::getPhi() const
{
    return this->angle();
}


mace_attitude_quaternion_t Rotation_2D::getMACEQuaternion() const
{
    mace_attitude_quaternion_t quat;

    return quat;
}

mace_attitude_t Rotation_2D::getMACEEuler() const
{
    mace_attitude_t euler;
    euler.yaw = static_cast<float>(this->getPhi());
    return euler;
}

mace_message_t Rotation_2D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_attitude_t attitude = getMACEEuler();
    mace_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}


} //end of namespace pose
} //end of namespace mace
