#ifndef COLLECT_AND_TIMEOUT_H
#define COLLECT_AND_TIMEOUT_H

#include "base.h"
#include "../types/collect-and-timeout.h"

#include "../../timer.h"


template <typename ...Rest>
class FramePersistanceBehavior<CollectAfterTimeout, Rest...> : public FramePersistanceBehavior<Rest...>
{
    static_assert(sizeof...(Rest) == 0, "Only one argument can be passed");

    Timer *m_Timer;
public:

    FramePersistanceBehavior(const CollectAfterTimeout & params) {
        m_Timer = new Timer(params.numMS, [this](){
            FramePersistanceBehavior<Rest...>::SendAndFinish();
        });
    }

    ~FramePersistanceBehavior() {
        delete m_Timer;
    }

    virtual void FrameReceived()
    {

    }
};

#endif // COLLECT_AND_TIMEOUT_H
