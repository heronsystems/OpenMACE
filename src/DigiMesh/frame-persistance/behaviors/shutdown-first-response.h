#ifndef SHUTDOWN_FIRST_RESPONSE_H
#define SHUTDOWN_FIRST_RESPONSE_H

#include "base.h"
#include "../types/shutdown-first-response.h"




template <typename ...Rest>
class FramePersistanceBehavior<ShutdownFirstResponse, Rest...> : public FramePersistanceBehavior<Rest...>
{
    static_assert(sizeof...(Rest) == 0, "Only one argument can be passed");
public:

    FramePersistanceBehavior(const ShutdownFirstResponse &params) {
    }

    void Test() {

    }


    virtual void FrameReceived()
    {
        FramePersistanceBehavior<Rest...>::SendAndFinish();
    }
};

#endif // SHUTDOWN_FIRST_RESPONSE_H
