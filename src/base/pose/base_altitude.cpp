#include "base_altitude.h"

namespace mace {
namespace pose {

Altitude::Altitude():
    Abstract_Altitude(), Position()
{

}

Altitude::Altitude(const Altitude &copy):
    Abstract_Altitude(copy), Position(copy)
{

}

//!
//! \brief setAltitude
//! \param altitude
//!
void Altitude::setAltitude(const double &altitude)
{
    this->z = altitude;
}

//!
//! \brief getAltitude
//! \return
//!
double Altitude::getAltitude() const
{
    return this->z;
}

void Altitude::applyTransformation(const Eigen::Transform<double, 2, Eigen::Affine> &t)
{
    UNUSED(t);
    //it does not make sense to apply a transformation here, and therefore will ignore.
}

void Altitude::applyTransformation(const Eigen::Transform<double, 3, Eigen::Affine> &t)
{
    Eigen::Vector3d currentData(0.0,0.0,this->z);

    //since this is only a 3D object we have to reconstruct
    Eigen::Vector3d result = t.linear() * currentData + t.translation();
    this->z = result.z();
}

mavlink_message_t Altitude::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    UNUSED(systemID);
    UNUSED(compID);
    UNUSED(chan);
    mavlink_message_t msg;
    return msg;
}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string Altitude::printPositionalInfo() const
{
    return "";
}

void Altitude::updateQJSONObject(QJsonObject &obj) const
{
    obj["alt"] = this->getAltitude();
}



} //end of namespace pose
} //end of namespace mace

