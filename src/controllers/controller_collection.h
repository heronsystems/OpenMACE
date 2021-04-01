#ifndef CONTROLLER_COLLECTION_H
#define CONTROLLER_COLLECTION_H

#include "unordered_map"
#include <mutex>
#include "I_controller.h"
#include "generic_controller.h"

#include <functional>

namespace Controllers {


//!
//! Container object that can be utalized to hold a collection of controllers
//!
template<typename MessageType, typename COMPONENT_KEY>
class ControllerCollection
{
private:

    std::unordered_map<std::string, IController<MessageType, COMPONENT_KEY>*> controllers;
    std::mutex controllerMutex;

public:


    ~ControllerCollection()
    {
        controllerMutex.lock();
        for(auto it = controllers.cbegin() ; it != controllers.cend() ; ++it)
        {
            controllers.erase(it->first);
            delete it->second;
        }
        controllerMutex.unlock();
    }

    IController<MessageType, COMPONENT_KEY>* At(const std::string &name)
    {
        try
        {
            return controllers.at(name);
        }
        catch (const std::out_of_range& e)
        {
            std::cerr <<"Inside the at of IController: "<< e.what() << std::endl;
            return nullptr;
        }

    }

    bool Exist(const std::string &name)
    {
        if(controllers.count(name) > 0)
            return true;
        else
            return false;
    }


    //!
    //! \brief Insert a pointer to controller
    //!
    //! When giving this object a pointer to a controller this object is taking responsibility for that pointer.
    //! \param name
    //! \param ptr
    //!
    void Insert(const std::string &name, IController<MessageType, COMPONENT_KEY>* ptr)
    {
        controllers.insert({name, ptr});
    }


    //!
    //! \brief Perform some action on each controller
    //! \param lambda Action to perform
    //!
    void ForAll(const std::function<void(IController<MessageType, COMPONENT_KEY>*)> &lambda)
    {
        controllerMutex.lock();
        for(auto it = controllers.cbegin() ; it != controllers.cend() ; ++it)
        {
            lambda(it->second);
        }
        controllerMutex.unlock();
    }


    //!
    //! \brief Present a received message to all controllers in the collection.
    //! \param msg Message to present to each controller
    //! \param key Object identifiying the sender of message
    //! \return True if message was consumed(used) by a controller
    //!
    bool Receive(const MessageType &msg, const COMPONENT_KEY &key)
    {
        bool consumed = false;

        controllerMutex.lock();
        for(auto it=controllers.begin(); it!=controllers.end(); ++it)
        {
            Controllers::IController<MessageType, COMPONENT_KEY>* obj = it->second;
            consumed = obj->ReceiveMessage(&msg, key);
        }
        controllerMutex.unlock();

        return consumed;
    }


    //!
    //! \brief Remove a controller and return the pointer
    //! \param name Name of controller to remove
    //! \return Pointer that was being stored
    //!
    IController<MessageType, COMPONENT_KEY>* Remove(const std::string &name)
    {
        IController<MessageType, COMPONENT_KEY>* ptr = nullptr;
        try
        {
            ptr = controllers.at(name);

            controllerMutex.lock();
            controllers.erase(name);
            controllerMutex.unlock();
        }
        catch (const std::out_of_range& e)
        {
            std::cerr <<"Inside the remove of IController: "<< e.what() << std::endl;
        }

        return ptr;
    }
};

}


#endif // CONTROLLER_COLLECTION_H
