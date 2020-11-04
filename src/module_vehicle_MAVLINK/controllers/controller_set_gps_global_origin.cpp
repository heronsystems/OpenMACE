#include "controller_set_gps_global_origin.h"

namespace MAVLINKUXVControllers {

void Controller_SetGPSGlobalOrigin::FillGPSOriginItem(const command_item::Action_SetGlobalOrigin &commandItem, mavlink_set_gps_global_origin_t &mavlinkItem)
{
    mace::pose::Abstract_GeodeticPosition* commandPosition = commandItem.getGlobalOrigin();
    mace::pose::GeodeticPosition_3D* castPosition = commandPosition->positionAs<mace::pose::GeodeticPosition_3D>();

    Data::EnvironmentTime currentTime;
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, currentTime);

    double power = std::pow(10,7);
    mavlinkItem.time_usec = currentTime.ToMillisecondsSinceEpoch() * 1000;
    mavlinkItem.latitude = castPosition->getLatitude() * power;
    mavlinkItem.longitude = castPosition->getLongitude() * power;
    mavlinkItem.altitude = castPosition->getAltitude() * 1000;
}

}// end of namespace MAVLINKVehicleControllers
