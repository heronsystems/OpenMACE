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

mace_message_t Altitude::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
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

