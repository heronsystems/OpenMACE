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

} //end of namespace pose
} //end of namespace mace

