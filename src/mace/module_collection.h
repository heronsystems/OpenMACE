#ifndef MODULE_COLLECTION_H
#define MODULE_COLLECTION_H
#include "module_external_link/module_external_link.h"

#include "module_ground_station/module_ground_station.h"

#include "module_path_planning_NASAPhase2/module_path_planning_nasaphase2.h"
#include "module_resource_task_allocation/module_rta.h"
#include "module_ROS/module_ROS.h"
#include "module_ROS_UMD/module_ROS_UMD.h"

#include "module_vehicle_sensors/module_vehicle_sensors.h"
#include "module_vehicle_arducopter/module_vehicle_arducopter.h"
#include "module_vehicle_arduplane/module_vehicle_arduplane.h"

#include "mace_core/module_factory.h"


class ModuleCollection : public MaceCore::ModuleFactory
{
public:

    static MaceCore::ModuleFactory* GenerateFactory()
    {
        MaceCore::ModuleFactory* factory = new MaceCore::ModuleFactory();
        Register<ModuleExternalLink>(factory, "Ardupilot");
        Register<ModuleGroundStation>(factory, "NASAPhase2");
        Register<ModulePathPlanningNASAPhase2>(factory, "NASAPhase2");
        Register<ModuleROS>(factory, "NASAPhase2");
        Register<ModuleROSUMD>(factory, "OFFSET_Auctioneer");
        Register<ModuleRTA>(factory, "NASAPhase2");
        Register<ModuleVehicleSensors>(factory, "NASAPhase2");
        Register<ModuleVehicleArducopter>(factory, "Arducopter");
        Register<ModuleVehicleArduplane>(factory, "Arduplane");

        return factory;
    }

private:

    template <typename T>
    static void Register(MaceCore::ModuleFactory* factory, const std::string &moduleName)
    {
        factory->RegisterFactory(T::moduleClass, moduleName, [](){return std::make_shared<T>();});
    }

    template <typename T>
    static void Register(MaceCore::ModuleFactory* factory)
    {
        factory->RegisterFactory(T::moduleClass, T::moduleName, [](){return std::make_shared<T>();});
    }
};

#endif // MODULE_COLLECTION_H
