#include "controller_set_gps_global_origin.h"

namespace MAVLINKUXVControllers {

void Controller_SetGPSGlobalOrigin::FillGPSOriginItem(const command_item::Action_SetGlobalOrigin &commandItem, mavlink_set_gps_global_origin_t &mavlinkItem)
{
    mace::pose::Abstract_GeodeticPosition* commandPosition = commandItem.getGlobalOrigin();
    mace::pose::GeodeticPosition_3D* castPosition = commandPosition->positionAs<mace::pose::GeodeticPosition_3D>();

    mavlinkItem.latitude = castPosition->getLatitude();
    mavlinkItem.longitude = castPosition->getLongitude();
    mavlinkItem.altitude = castPosition->getAltitude();
}

}// end of namespace MAVLINKVehicleControllers
