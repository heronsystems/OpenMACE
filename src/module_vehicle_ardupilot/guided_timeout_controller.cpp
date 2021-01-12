#include "guided_timeout_controller.h"

namespace ardupilot_vehicle {

GuidedTimeoutController::GuidedTimeoutController(const unsigned int &timeout, const int &levelTimeout, const int &killTimeout)
{
    m_CBTarget = nullptr;
    m_FunctionTarget = nullptr;

    this->timeout = timeout;
    this->levelTimeout = levelTimeout;
    this->abortTimeout = killTimeout;
}

GuidedTimeoutController::GuidedTimeoutController(const GuidedTimeoutController &copy)
{
    m_CBTarget = nullptr;
    m_FunctionTarget = nullptr;

    this->timeout = copy.timeout;
    this->levelTimeout = copy.levelTimeout;
    this->abortTimeout = copy.abortTimeout;
}


GuidedTimeoutController::~GuidedTimeoutController() {
    std::cout << "Destructor on guided timeout controller" << std::endl;
    this->stop();
}

void GuidedTimeoutController::registerAbortingTarget(const command_item::AbstractCommandItemPtr command, const TimeoutMode &mode, const int &timeout)
{
    UNUSED(command);
    UNUSED(mode);
    UNUSED(timeout);
}

void GuidedTimeoutController::registerCurrentTarget(const command_item::Action_DynamicTarget &commandTarget, const TimeoutMode &mode)
{
    if(isThreadActive())
    {
        m_LambdasToRun.push_back([this,commandTarget, mode]{
            switch (mode) {
            case TimeoutMode::NORMAL:
            {
                this->m_Timeout.stop();
                this->m_CurrentTarget = commandTarget;
                this->m_Timeout.start();
                break;
            }
            case TimeoutMode::ABORT:
            case TimeoutMode::LEVELING:
            default:
            {
                break;
            }
            }
        });
    }
    else
    {
        switch (mode) {
        case TimeoutMode::NORMAL:
        {
            this->m_CurrentTarget = commandTarget;
            this->m_Timeout.start();
            Thread::start();

            break;
        }
        case TimeoutMode::ABORT:
        case TimeoutMode::LEVELING:
        default:
        {
            break;
        }
        }
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
