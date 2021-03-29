#ifndef USAGE_H
#define USAGE_H

#include "controller.h"

#include <thread>
#include <functional>


class ExampleControllerUsage : public std::thread, public Controllers::IMessageNotifier<CommPacket, ExampleEntity>
{
private:

    ExampleController *exampleController;

    ExampleEntity m_Entity;

public:

    std::function<void(const CommPacket &msg, const OptionalParameter<ExampleEntity> &target)> m_TransmitLambda;

    void Receive(const CommPacket &msg)
    {
        exampleController->ReceiveMessage(&msg, msg.sender);
    }

public:

    ExampleControllerUsage(const ExampleEntity &entity) :
        m_Entity(entity)
    {
        TransmitQueue *queue = new TransmitQueue(4000, 3);
        exampleController = new ExampleController(this, queue, 0);
    }


    void AddResource(uint8_t ID, const std::string &name, const ExampleEntity &target)
    {
        ResourceIdentifier RID;
        RID.ID = ID;

        ResourceData RData;
        RData.name = name;
        exampleController->Send(std::make_tuple(RID, RData), m_Entity, target);
    }




    //!
    //! \brief TransmitMessage
    //! \param msg Message to transmit
    //! \param target Target to transmitt to. Broadcast if not set.
    //!
    virtual void TransmitMessage(const CommPacket &msg, const OptionalParameter<ExampleEntity> &target) const
    {
        printf("Transmitting a packet\n");
        m_TransmitLambda(msg, target);
    }


    //!
    //! \brief GetKeyFromSecondaryID
    //! \param ID
    //! \return
    //!
    virtual ExampleEntity GetKeyFromSecondaryID(int ID) const
    {
        throw std::runtime_error("Unused!");
    }

    virtual ExampleEntity GetHostKey() const
    {
        return m_Entity;
    }



};



#endif // USAGE_H
