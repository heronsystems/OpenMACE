#include "guided_timeout_controller.h"

namespace ardupilot_vehicle {

GuidedTimeoutController::GuidedTimeoutController(const unsigned int &timeout)
{
    m_CBTarget = nullptr;
    m_FunctionTarget = nullptr;

    this->timeout = timeout;
}

GuidedTimeoutController::~GuidedTimeoutController() {
    std::cout << "Destructor on guided timeout controller" << std::endl;
    this->stop();
}

void GuidedTimeoutController::registerCurrentTarget(const command_target::DynamicTarget &target)
{
    if(isThreadActive())
    {
        m_LambdasToRun.push_back([this,target]{
            this->m_Timeout.stop();
            this->m_CurrentTarget = target;
            this->m_Timeout.start();
        });
    }
    else
    {
        this->m_CurrentTarget = target;
        this->m_Timeout.start();
        Thread::start();
    }
}

void GuidedTimeoutController::clearTarget()
{
    this->stop();
}

void GuidedTimeoutController::start()
{
    this->m_Timeout.start();
    Thread::start();
}

void GuidedTimeoutController::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
            m_Timeout.stop();
            break;
        }

        this->RunPendingTasks();

        //The current state we can find out how much time has passed.
        //If one of the lambda expressions has fired the clock should
        //be reset right at the end, thus making this value small and
        //improbable the next function will fire
        double timeElapsed = m_Timeout.elapsedMilliseconds();

        if(timeElapsed >= timeout)
        {
            this->callTargetCallback(m_CurrentTarget);
            m_Timeout.reset();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(timeout/2));
    }
}



} //end of namespace ardupilot_vehicle
