/*!
  * @file vehicle_object_arduplane.cpp
  *
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *     This is a part of Heron Systems MACE ecosystem.
  *
  * @section DESCRIPTION
  *
  * @date
  *     August 2020
  *
  * @copyright
  *     File and its related contents are subjected to a software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#include "vehicle_object_arduplane.h"

VehicleObject_Arduplane::VehicleObject_Arduplane(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    VehicleObject_Ardupilot(commsObj, module, mavlinkID)
{
    m_ArdupilotMode = new ARDUPLANEComponent_FlightMode();
}

VehicleObject_Arduplane::~VehicleObject_Arduplane()
{
    if(m_ArdupilotMode != nullptr)
        delete m_ArdupilotMode;
}
