///
/// Example of various parts of MACE
///
/// This example is somewhat limited because the example module can not be physically attached to the Core.
/// This is because the module does not characterize one of the pre-defined module type the core is expecting.
///
/// As an alternative, each part to be covered will be placed in-line. In true operation much of what is displayed here would be inside its own module.
///
#include <iostream>
#include <thread>

#include "mace_core/mace_core.h"


#include "example_module.h"
#include "data_interpolation.h"

#include "example_controller/usage.h"

#include "example_topics.h"

#include "example_numerical_analysis.h"

using namespace std;

int main(int argc, char *argv[])
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////

    MaceCore::MaceCore core;
    std::shared_ptr<MaceCore::MaceData> data = std::make_shared<DataInterpolation>();
    core.AddDataFusion(data);

    /*

    /// Create Example Module
    ExampleMetaData metaData;
    std::shared_ptr<ModuleExample> exampleModule = std::make_shared<ModuleExample>();
    exampleModule->setDataObject(data);



    /// Create configuration for new module, This would be read in from some config file or given based on comand line arguments
    std::shared_ptr<MaceCore::ModuleParameterValue> configuration = std::make_shared<MaceCore::ModuleParameterValue>();
    configuration->AddTerminalValue("Int1", 5);

    std::shared_ptr<MaceCore::ModuleParameterValue> nest1 = std::make_shared<MaceCore::ModuleParameterValue>();
    nest1->AddTerminalValue("Nest1_Int1", 1);
    nest1->AddTerminalValue("Nest1_Int2", 2);
    nest1->AddTerminalValue("Nest1_Double1", 0.25);
    configuration->AddNonTerminal("Nest1", nest1);


    /// Configure module
    exampleModule->ConfigureModule(configuration);

    */


    //normally would add module to core
    //core.AddRTAModule(rtaModule);

    //////////////////////////////////////////////////////////////////////////////////////////////
    /// NUMERICAL ANALYSIS EXAMPLE
    //////////////////////////////////////////////////////////////////////////////////////////////


    NumericalAnalysis::NumericalAnalysisContainer<TestEnvironment, TestSolutionMetrics> container(std::chrono::seconds(10));

    TestEnvironment environment;
    environment.min = -10;
    environment.max = 10;

    container.SetEnvironment(environment);

    container.Start(std::make_shared<TestAlgorithm>(), std::make_shared<TestConditions>(), [](std::shared_ptr<NumericalAnalysis::IProblemOutcome> outcome){
        printf("Outcome Determined!\n");

        if(outcome->Type() == NumericalAnalysis::TerminationTypes::CONVERGED)
        {
            std::shared_ptr<NumericalAnalysis::ProblemOutcome_Converged<TestSolutionMetrics>> ptr = std::dynamic_pointer_cast<NumericalAnalysis::ProblemOutcome_Converged<TestSolutionMetrics>>(outcome);
            printf("  Converged to %f -> %f\n", ptr->Solution().x, ptr->Solution().value);
        }
        if(outcome->Type() == NumericalAnalysis::TerminationTypes::TIMEOUT)
        {
            printf("  Timed out\n");
        }
        if(outcome->Type() == NumericalAnalysis::TerminationTypes::ABORTED)
        {
            std::shared_ptr<NumericalAnalysis::ProblemOutcome_Aborted> ptr = std::dynamic_pointer_cast<NumericalAnalysis::ProblemOutcome_Aborted>(outcome);
            printf("  Aborted: %s\n", ptr->Reason().c_str());
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(10));

    //////////////////////////////////////////////////////////////////////////////////////////////
    /// CONTROLLER CREATION AND USAGE EXAMPLE
    //////////////////////////////////////////////////////////////////////////////////////////////

    ExampleEntity e1;
    e1.ID = 1;

    ExampleEntity e2;
    e2.ID = 2;



    ExampleControllerUsage *exampleControllerSender = new ExampleControllerUsage(e1);
    ExampleControllerUsage *exampleControllerReceiver = new ExampleControllerUsage(e2);


    //! this lambda takes the place of some communication stratagy like DigiMesh
    auto dispatchMessages = [exampleControllerSender, exampleControllerReceiver, e1, e2](const CommPacket &msg, const OptionalParameter<ExampleEntity> &target)
    {
        //we have to move to thread since we are only operating on single process in this example
        //Normally the communication stratagy would cause seperate threads/processes/machines naturally.
        std::thread thread([=](){
            if(target().ID == e1.ID)
            {
                exampleControllerSender->Receive(msg);
            }
            if(target().ID == e2.ID)
            {
                exampleControllerReceiver->Receive(msg);
            }
        });
        thread.detach();
    };

    exampleControllerSender->m_TransmitLambda = dispatchMessages;
    exampleControllerReceiver->m_TransmitLambda = dispatchMessages;

    exampleControllerSender->AddResource(1, "Test", e2);



    //////////////////////////////////////////////////////////////////////////////////////////////
    /// TOPIC EXAMPLE
    //////////////////////////////////////////////////////////////////////////////////////////////


    std::shared_ptr<TopicComponent_Attitude> attitudeDataSend = std::make_shared<TopicComponent_Attitude>(10.0, 20.0, 30.0);
    MaceCore::TopicDatagram topicDatagram;
    TopicType_Telemetry::SetComponent(attitudeDataSend, topicDatagram);

    std::shared_ptr<TopicComponent_Attitude> attitudeDataRcv = std::make_shared<TopicComponent_Attitude>();
    TopicType_Telemetry::GetComponent(topicDatagram, attitudeDataRcv);

    std::cout << attitudeDataRcv->Roll() << " " << attitudeDataRcv->Pitch() << " " << attitudeDataRcv->Yaw() << std::endl;


    while(true)
    {

    }


}
