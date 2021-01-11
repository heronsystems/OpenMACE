#include <iostream>
#include <thread>

#include <QDir>
#include <QString>

#include "mace_core/mace_core.h"

#include "configuration_reader_xml.h"

#include "data_interpolation.h"

#include "module_collection.h"

#include "data/environment_time.h"

#include "mace_core/module_characteristics.h"

#ifdef ROS_EXISTS
#include <ros/ros.h>
#endif

const char kPathSeparator =
#ifdef _WIN32
                            '\\';
#else
                            '/';
#endif

int main(int argc, char *argv[])
{
    Data::EnvironmentTime currentTime;
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, currentTime);

//    currentTime.ToString();
    //generate the factory that can make module instances
    MaceCore::ModuleFactory* factory = ModuleCollection::GenerateFactory();

    //Initialize core and configure data object
    MaceCore::MaceCore core;
    std::shared_ptr<MaceCore::MaceData> data = std::make_shared<DataInterpolation>();
    core.AddDataFusion(data);

    std::string filename = "";

    char* MACEPath = getenv("MACE_ROOT");
    std::string loggingPath = "";
    if(MACEPath){
        std::string rootPath(MACEPath);
        //rootPath += "/logs/";
        QDir loggingDirectory(QString::fromStdString(rootPath + "/logs/"));

        std::string newPath = currentTime.dateString() + "_Test_";
        int testIndex = 0;
        std::string finalPath = newPath + std::to_string(testIndex);

        loggingDirectory.mkpath(QString::fromStdString(rootPath + "/logs/"));
        while(!loggingDirectory.mkdir(QString::fromStdString(finalPath)))
        {
            testIndex++;
            finalPath = newPath + std::to_string(testIndex);
        }
        loggingPath = loggingDirectory.absolutePath().toStdString() + "/" + finalPath;

        std::cout << "The current MACE_ROOT path is: " << rootPath << std::endl;
        filename = rootPath + kPathSeparator + "MaceSetup_Configs" + kPathSeparator + "Default.xml";
    }else{
        filename = "MaceSetup_Configs/Default.xml";
    }

    if(argc >= 2){
        std::string rootPath(MACEPath);
        filename = rootPath + kPathSeparator + argv[1];
        //filename = argv[1];
    }

#ifdef ROS_EXISTS
    std::map<std::string, std::string> remappings;
    ros::init(remappings,"ROS_Module");
#endif

    std::cout << "Reading MACE configuration file from: " << filename << std::endl;

    ConfigurationReader_XML parser(factory);
    ConfigurationParseResult parseResult = parser.Parse(filename);
    if(parseResult.success == false)
    {
        std::cerr << "Error parsing configuration file: " << std::endl;
        std::cerr << parseResult.error << std::endl;
        return 1;
    }

    if(parseResult.warnings.size() > 0)
    {
        std::cout << "Configuration Parse Warnings:" << std::endl;
        for(auto it = parseResult.warnings.cbegin() ; it != parseResult.warnings.cend() ; ++it)
            std::cout << *it << std::endl;
    }

    bool addedGroundStation = false;
    bool addedPathPlanning = false;
    bool addedROS = false;
    bool addedGlobalRTA = false;
    bool addedSensors = false;
    int numVehicles = 1;

    // If a static address is given in config then distribute out
    int hostMaceInstance;
    if(parser.HasStaticMaceInstanceID() == true)
    {
        hostMaceInstance = parser.GetStaticMaceInstanceID();;
    }
    else {
        throw std::runtime_error("No Mace instance ID is given.");
        //MTB - Unimplimented funcationalty. If no ID is given MACE can determine an ID to use. This would require coordination to avoid conflicts.
    }

    core.setGlobalConfiguration(parser.GetGlobalConfiguration()); // Send global parameters to mace_core for setup

    core.setMaceInstanceID(hostMaceInstance);//to be removed?

    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string > modules = parser.GetCreatedModules();
    std::vector<std::thread*> threads;
    std::vector<int> reservedIDs;

    /// Configure modules
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        std::shared_ptr<MaceCore::ModuleBase> module = it->first;
        std::string moduleType = it->second;
        std::cout << "Creating a " << MaceCore::ModuleBase::ModuleTypeToString(module->ModuleClass()) << " module of type: " << moduleType << std::endl;

        module->AssignLoggingDirectory(loggingPath);

        //set data object of module
        module->setDataObject(data);

        //configure module
        module->ConfigureModule(parser.GetModuleConfiguration(module));

        if(module->HasID() == true)
        {
            reservedIDs.push_back(module->GetID());
        }
    }


    /// Set the reserved ID's in the core so it doesn't assign an already assigned module
    core.setReservedIDs(reservedIDs);


    /*
    /// More set up that requires every module to be configured
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        std::shared_ptr<MaceCore::ModuleBase> module = it->first;
        MaceCore::ModuleClasses moduleClass = module->ModuleClass();
        switch (moduleClass) {
            case MaceCore::ModuleClasses::RTA:
            {
                std::shared_ptr<MaceCore::IModuleCommandRTA> m = std::dynamic_pointer_cast<MaceCore::IModuleCommandRTA>(module);

                m->GetMetaData();
            }
            default:
            {
                continue;
            }
        }
    }
    */


    /// Add modules to core
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        std::shared_ptr<MaceCore::ModuleBase> module = it->first;
        MaceCore::ModuleClasses moduleClass = module->ModuleClass();
        switch (moduleClass) {
        case MaceCore::ModuleClasses::EXTERNAL_LINK:
        {
            const std::shared_ptr<MaceCore::IModuleCommandExternalLink> externalLink = std::dynamic_pointer_cast<MaceCore::IModuleCommandExternalLink>(module);

            //notify external link of the attached mace instance
            /*
            MaceCore::ModuleCharacteristic hostMaceInstance;
            hostMaceInstance.ID = MaceInstanceStaticID;
            hostMaceInstance.Class = MaceCore::ModuleClasses::MACE_INSTANCE;
            externalLink->MarshalCommand<MaceCore::ModuleCharacteristic>(MaceCore::ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, hostMaceInstance);
            */

            //add external link to core
            core.AddLocalModule_ExternalLink(externalLink);
            break;
        }
        case  MaceCore::ModuleClasses::GROUND_STATION:
        {
            if(addedGroundStation == true)
            {
                std::cerr << "Only one Ground Station module can be added" << std::endl;
                return 1;
            }
            core.AddGroundStationModule(std::dynamic_pointer_cast<MaceCore::IModuleCommandGroundStation>(module));
            addedGroundStation = true;
            break;
        }
        case MaceCore::ModuleClasses::SENSORS:
        {
            if(addedSensors == true)
            {
                std::cerr << "Only one sensors module can be added" << std::endl;
                return 1;
            }
            core.AddSensorsModule(std::dynamic_pointer_cast<MaceCore::IModuleCommandSensors>(module));
            addedSensors = true;
            break;
        }
        case MaceCore::ModuleClasses::PATH_PLANNING:
        {
            if(addedPathPlanning == true)
            {
                std::cerr << "Only one path planning module can be added" << std::endl;
                return 1;
            }
            core.AddPathPlanningModule(std::dynamic_pointer_cast<MaceCore::IModuleCommandPathPlanning>(module));
            addedPathPlanning = true;
            break;
        }

        case MaceCore::ModuleClasses::ROS:
        {
#ifdef ROS_EXISTS
            if(addedROS == true)
            {
                std::cerr << "Only one ROS module can be added" << std::endl;
                return 1;
            }
            core.AddROSModule(std::dynamic_pointer_cast<MaceCore::IModuleCommandROS>(module));
            addedROS = true;
#else
            std::cout << "ROS not included on this system. Skipping ROS module setup." << std::endl;
#endif
            break;
        }
        case MaceCore::ModuleClasses::RTA:
        {


            std::shared_ptr<MaceCore::IModuleCommandRTA> m = std::dynamic_pointer_cast<MaceCore::IModuleCommandRTA>(module);
            if(m->getModuleMetaData().IsGlobal() == true)
            {
                if(addedGlobalRTA == true)
                {
                    std::cerr << "Only one global RTA module can be added" << std::endl;
                    return 1;
                }

                core.AddLocalModule_GlobalRTA(m);
                addedGlobalRTA = true;
            }
            else
            {
                core.AddLocalModule_SpecializedRTA(m);
            }

            break;
        }
        case MaceCore::ModuleClasses::VEHICLE_COMMS:
        {
            core.AddLocalModule_Vehicle(std::to_string(numVehicles), std::dynamic_pointer_cast<MaceCore::IModuleCommandVehicle>(module));
            numVehicles++;
            break;
        }
        default:
        {
            std::cerr << "Unknown module parsed" << std::endl;
            return 1;
        }
        }

        if(moduleClass != MaceCore::ModuleClasses::ROS)
        {
            core.Event_NewModule(module.get(), module->GetCharacteristic(), moduleClass);
        }
        else {
#ifdef ROS_EXISTS
            core.Event_NewModule(module.get(), module->GetCharacteristic(), moduleClass);
#endif
        }
    }

    /// Start Modules
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        std::shared_ptr<MaceCore::ModuleBase> module = it->first;
        std::thread *thread = new std::thread([module]()
        {
            module->start();
        });
        threads.push_back(thread);
    }

    /// Signal start
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        std::shared_ptr<MaceCore::ModuleBase> module = it->first;
        /*
         * This will notify all the modules that all of the other modules are ready
         * at this time they should grab anything that another module may have defined
         * in the core and begin operating.
         */
        module->OnModulesStarted();

    }


    //////
    ///// Madison Memory Testing : Force shutdown of all modules after some time
    /////
    /*
    static const int TIME_TO_SHUTDOWN = 90;
    printf("!!!!!!!!!!!WARNING!!!!!!!!!!!\n  -- Debug code will be forcing shutdown of MACE after %d seconds  -- \n", TIME_TO_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(TIME_TO_SHUTDOWN* 1000));
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        it->first->shutdown();
    }
    */
    /// End Madison Testing

    //wait for all threads to complete
    for(std::thread* thread: threads)
    {
        thread->join();
        delete thread;
    }



    delete factory;

    return 0;
}
