#ifndef EXAMPLE_MODULE_H
#define EXAMPLE_MODULE_H

#include <iostream>

#include "common/optional_parameter.h"
#include "mace_core/abstract_module_event_listeners.h"


//#define EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP

/*
 * This file creates an example module for demonstration of Module functionality.
 *
 */


class ExampleMetaData
{

};


//!
//! \brief Interface defining the events that module is to generate.
//!
//! For modules used by mace this object will be in MaceCore as one of the "i_module_events_<moduleName>.h"
//!
class ExampleModuleEvents
{
    virtual void Event1() = 0;

    virtual void Event2(const int&) = 0;
};

//!
//! \brief Enumeration of commands that the module is to generate
//!
enum class ExampleCommands
{
    Command1
};


//!
//! \brief Interface for an Modules commands.
//!
//! The interface exposes a public set of methods which implimentor must instantiate in order to use the module.
//! Mace's module interfaces are the set of objects described by: MaceCore::IModuleCommand<moduleName>.
//!
//! It for there to exists multiple modules from the same Interface (and of same class) that impliment behavior differently.
//! Therefore Mace's module interfaces describe how each module class should behave without implimenting the specific behavior.
//!
class IModuleCommandExample : public MaceCore::AbstractModule_EventListeners<ExampleMetaData, ExampleModuleEvents, ExampleCommands>
{

public:
    IModuleCommandExample() :
        MaceCore::AbstractModule_EventListeners<ExampleMetaData, ExampleModuleEvents, ExampleCommands>()
    {
        //configure the module to know that when ExampleCommands::Command1 is commanded Command1() should be called
        AddCommandLogic(ExampleCommands::Command1, [this](const OptionalParameter<MaceCore::ModuleCharacteristic> &sender){
            Command1();
        });
    }

    virtual MaceCore::ModuleClasses ModuleClass() const
    {
        return MaceCore::ModuleClasses::NR_TYPES;
    }


    //!
    //! \brief Abstract command that is to be invoked when "Command1" command is issued
    //!
    virtual void Command1() = 0;

};



//!
//! \brief Implimentation of the Example Module
//!
//!
//! The implementation of a module shall contain the logic that is to be done with a command is invoked.
//! Each module exists on it's own thread, which logic can be marshaled on.
//!
//!
//!
//! Commands can be configured to either be executed asyncronously or syncronously.
//! By default all commands are configured to be asyncronous.
//!
//! If configured asyncronously, the virtual methods in module's interface will be invoked on the modules thread.
//! If configured syncronously, the virtual methods in module's interface will be invoked not on the modules thread and may require custom marshaling to have code ran on modules thread.
//! Custom marshaling will itself require the start() method to be overridden.
//!
//! In this example module the presence of EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP will make Command1 behave syncrounsouly.
//! While if EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP is not defined it will behave asyncrounsly.
//!
//!
//!
//! Each concrete module has its own configuration parameter structure and each instance can be configured by a unique set of parameters.
//! The structure of parameters the module is expected is provided by the modules implimentation of ModuleConfigurationStructure
//! The structure is a context-free object that can contain classes of parameters
//! For example a vehicle comms module can contain a parameter class that describes the protocol to be used, and another to describe the specific serial link
//!
//! At runtime the module is configured by calling ConfigureModule.
//! In this method, the module will go through the known structure of parameters and create nessessary objects for module operation
//!
//!
//!
//! Data from other modules that is needed may be stored in the MaceCore::MaceData object defined globally and passed to each module.
//! To access the data object use getDataObject method.
//! The reason for a seperate "global" data container over simply passing data in command methods was done for three reasons:
//!     1) Some modules may require the same data, so a centrailized container is simplier over distrubtiting idential data.
//!     2) Some data may be updated at a high rate, so changing one and calling a notify modules of new data is prefered over sending new data
//!     3) Commands may be marsheled at a different rate than they are generated, so a centralize data bin gives the module full access to data without over-invoking
//! Each module has read access to the MaceData object, but not write access.
//!
//!
//!
//! When it comes time the module can call events to anyone listening.
//! The events that can be generated by an event are described in the IModuleEvent<moduleName> that is passed as template to AbstractModule_EventListners
//! Objects subscribe to events by calling addListener and passing the listnener object.
//! Inside the module, an event can be invoked on all listeners using the NotifyListeners method.
//!
class ModuleExample : public IModuleCommandExample
{

public:

#ifdef EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP
    ModuleExample() :
        IModuleCommandExample(),
        Command1Issued(false)
    {
        //set such that all commands invoked syncrounously (i.e. not on module's thread)
        this->InvokeCommandOnModuleEventLoop(false);

        //Alternativly it can be configured such that only a subset of commands be invoked syncronously
        this->InvokeCommandOnModuleEventLoop(ExampleCommands::Command1, false);
    }

#else
    ModuleExample() :
        IModuleCommandExample()
    {
    }
#endif



    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const
    {
        MaceCore::ModuleParameterStructure structure;

        MaceCore::ModuleParameterStructure nest1;
        nest1.AddTerminalParameters("Nest1_Int1", MaceCore::ModuleParameterTerminalTypes::INT);
        nest1.AddTerminalParameters("Nest1_Int2", MaceCore::ModuleParameterTerminalTypes::INT);
        nest1.AddTerminalParameters("Nest1_Double1", MaceCore::ModuleParameterTerminalTypes::DOUBLE);

        structure.AddNonTerminal("Nest1", std::make_shared<MaceCore::ModuleParameterStructure>(nest1));

        structure.AddTerminalParameters("Int1", MaceCore::ModuleParameterTerminalTypes::INT);

        return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
    }


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
    {
        int int1 = params->GetTerminalValue<int>("Int1");
        std::cout << "Int1 " << int1 << std::endl;

        std::shared_ptr<MaceCore::ModuleParameterValue> nest1 = params->GetNonTerminalValue("Nest1");
        std::cout << "Nest1 Int1 " << nest1->GetTerminalValue<int>("Nest1_Int1") << std::endl;
        std::cout << "Nest1 Int2 " << nest1->GetTerminalValue<int>("Nest1_Int2") << std::endl;
        std::cout << "Nest1 Double1 " << nest1->GetTerminalValue<double>("Nest1_Double1") << std::endl;
    }

#ifdef EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP

    //!
    //! \brief Overridden event loop that will check for custom marshaling
    //!
    virtual void start()
    {
        while(true)
        {
            //check if a new command was introduced
            if(Command1Issued == true)
            {
                std::cout << "Command1 in module's thread" << std::endl;
                Command1Issued = false;
            }

            //check for any commands that may be still configured to execute asyncrounously
            ExecuteQueuedCommands();

            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
    }

#endif


    virtual void Command1()
    {
#ifdef EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP
        std::cout << "Command1 called, must be marshaled onto event loop" << std::endl;
        Command1Issued = true;
#else
        std::cout << "Command1 in module's thread" << std::endl;
#endif
    }

private:

#ifdef EXAMPLE_MODULE_BEHAVIOR_CUSTOM_EVENT_LOOP
    bool Command1Issued;
#endif

};


#endif // EXAMPLE_MODULE_H
