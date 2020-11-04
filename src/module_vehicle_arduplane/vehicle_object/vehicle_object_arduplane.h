/*!
  * @file vehicle_object_arduplane.h
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

#ifndef VEHICLE_OBJECT_ARDUPLANE_H
#define VEHICLE_OBJECT_ARDUPLANE_H

#include "module_vehicle_ardupilot/vehicle_object/vehicle_object_ardupilot.h"

#include "arduplane_component_flight_mode.h"

//!
//! \brief The VehicleObject_Arduplane class is the implementation of an explicit object inheriting the base vehicle object from the ardupilot code base.
//! The purpose of this class is to act as a container for the vehicle data contained within the module.
//!
//!

class VehicleObject_Arduplane : public VehicleObject_Ardupilot
{
public:
    VehicleObject_Arduplane(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID);

    ~VehicleObject_Arduplane();
};


#endif // VEHICLE_OBJECT_ARDUPLANE_H
