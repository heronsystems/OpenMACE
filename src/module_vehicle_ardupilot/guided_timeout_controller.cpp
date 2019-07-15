#include "guided_timeout_controller.h"

namespace mavlink {


GuidedTimeoutController::GuidedTimeoutController(ArdupilotTimeout_Interface* callback, const unsigned int &timeout):
    currentTarget(nullptr)
{
    this->m_CB = callback;
    this->timeout = timeout;
}

GuidedTimeoutController::~GuidedTimeoutController() {
    std::cout << "Destructor on guided timeout controller" << std::endl;
    mToExit = true;
}

void GuidedTimeoutController::start() override
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
            //For the moment we are removing the callback timeout
            if((currentTarget != nullptr) && (m_CB != nullptr))
                m_CB->cbiArdupilotTimeout_TargetLocal(*currentTarget);
            m_Timeout.reset();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(timeout/2));
    }
}

void GuidedTimeoutController::updateTarget(const TargetItem::CartesianDynamicTarget &target)
{
    m_LambdasToRun.push_back([this, target]{
        currentTarget = new TargetItem::CartesianDynamicTarget(target);
        //reset the timeout and send this new command
    });
}

void GuidedTimeoutController::clearTarget()
{
    delete currentTarget;
    currentTarget = nullptr;
}

void GuidedTimeoutController::setCallbackFunction(ArdupilotTimeout_Interface* callback)
{
    m_CB = callback;
}

} //end of namespace mavlink
