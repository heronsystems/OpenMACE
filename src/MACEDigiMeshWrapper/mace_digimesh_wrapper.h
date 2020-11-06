#ifndef MACE_WRAPPER_H
#define MACE_WRAPPER_H

#include "macewrapper_global.h"

#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <stdint.h>
#include <mutex>
#include <thread>
#include <functional>

#include "digi_common/digi_mesh_baud_rates.h"
#include "digi_common/transmit_status_types.h"
#include "component.h"

#include "interop_component.h"


template<const char*... A>
class _MACEDigiMeshWrapper;


template<const char* Type, const char*... Rest>
class _MACEDigiMeshWrapper<Type, Rest...> : public _MACEDigiMeshWrapper<Rest...>
{
private:

public:

    _MACEDigiMeshWrapper<Type, Rest...>()
    {
        _MACEDigiMeshWrapper<Rest...>::m_ElementMap.insert({Type, new Component()});
    }

    ~_MACEDigiMeshWrapper<Type, Rest...>()
    {

    }

};


template <>
class MACEWRAPPERSHARED_EXPORT _MACEDigiMeshWrapper<>
{
public:
    std::unordered_map<std::string, Component*> m_ElementMap;

public:

    _MACEDigiMeshWrapper<>() {

    }
};

template <const char*... A>
class MACEWRAPPERSHARED_EXPORT MACEDigiMeshWrapper : public InteropComponent, public _MACEDigiMeshWrapper<A...>
{
    //using _MACEDigiMeshWrapper<A...>::m_ElementMap;

public:


    MACEDigiMeshWrapper(const std::string &port, DigiMeshBaudRates rate, const std::string &nameOfNode = "", bool scanForNodes = false) :
        InteropComponent(port, rate, nameOfNode, scanForNodes)
    {

        //broadcast a request for everyone to send their Elements
        variadicExpand<A...>([this](const char* element) {

            InteropComponent::RequestContainedResources({element});
        });

    }

    virtual ~MACEDigiMeshWrapper()
    {
        // TODO: Anything to be done here?
    }


    void RequestRemoteResources() const
    {
        variadicExpand<A...>([this](const char* element) {

            InteropComponent::RequestContainedResources({element});
        });

    }



//    /**
//     * @brief Add handler to be called when a new vehicle is added to the network
//     * @param lambda Lambda function whoose parameters are the vehicle ID and node address of new vechile.
//     */
//    template <const char* T>
//    void AddHandler_NewRemoteComponentItem(const std::function<void(int, uint64_t)> &lambda)
//    {
//        InteropComponent::AddHandler_NewRemoteComponentItem(T, lambda);
//    }


//    /**
//     * @brief Add handler to be called when a new vehicle has been removed from the network
//     * @param lambda Lambda function whoose parameters are the vehicle ID of removed vechile.
//     */
//    template <const char* T>
//    void AddHandler_RemovedRemoteComponentItem(const std::function<void(int)> &lambda)
//    {
//        InteropComponent::AddHandler_RemoteComponentItemRemoved(T, lambda);
//    }


//    /**
//     * @brief Add handler to be called when tranmission to a vehicle failed for some reason.
//     * @param lambda Lambda function to pass vehicle ID and status code
//     */
//    template <const char* T>
//    void AddHandler_ComponentItemTransmitError(const std::function<void(int vehicle, TransmitStatusTypes status)> &lambda)
//    {
//        InteropComponent::AddHandler_ComponentItemTransmitError(T, lambda);
//    }


    void AddResource(const ResourceKey &key, const ResourceValue &value) // recursive variadic function
    {
        InteropComponent::AddResource(key, value);
    }


    template<const char* ...str, typename... Args>
    void AddResource(Args... args) // recursive variadic function
    {
        static_assert(sizeof...(str) == sizeof...(Args), "Name and Resource values must be the same");



        std::vector<int> vec = _BuildIntVector(args...) ;
        std::vector<std::string> names = {str...};

        InteropComponent::AddResource(ResourceKey(names), ResourceValue(vec));
    }


    bool HasResource(const ResourceKey &key, const ResourceValue &value) // recursive variadic function
    {
        return InteropComponent::HasResource(key, value);
    }


    bool SendData(const std::vector<uint8_t> &data, const ResourceKey &key, const ResourceValue &value) // recursive variadic function
    {
        return InteropComponent::SendData(key, value, data);
    }

    template<const char* ...str, typename... Args>
    bool SendData(const std::vector<uint8_t> &data, Args... args) // recursive variadic function
    {
        static_assert(sizeof...(str) == sizeof...(Args), "Name and Resource values must be the same");

        std::vector<int> vec = _BuildIntVector(args...) ;
        std::vector<std::string> names = {str...};
        return InteropComponent::SendData(ResourceKey(names), ResourceValue(vec), data);
    }

    /**
     * @brief Add handler to be called when a new vehicle is added to the network
     * @param lambda Lambda function whoose parameters are the vehicle ID and node address of new vechile.
     */
    void AddHandler_NewRemoteComponentItem_Generic(const std::function<void(ResourceKey, ResourceValue, uint64_t)> &lambda)
    {
        InteropComponent::AddHandler_NewRemoteComponentItem_Generic(lambda);
    }


    /**
     * @brief Add handler to be called when a new vehicle has been removed from the network
     * @param lambda Lambda function whoose parameters are the vehicle ID of removed vechile.
     */
    void AddHandler_RemovedRemoteComponentItem_Generic(const std::function<void(ResourceKey, ResourceValue)> &lambda)
    {
        InteropComponent::AddHandler_RemoteComponentItemRemoved_Generic(lambda);
    }


    /**
     * @brief Add handler to be called when tranmission to a vehicle failed for some reason.
     * @param lambda Lambda function to pass vehicle ID and status code
     */
    void AddHandler_ComponentItemTransmitError_Generic(const std::function<void(ResourceKey, ResourceValue, TransmitStatusTypes status)> &lambda)
    {
        InteropComponent::AddHandler_ComponentItemTransmitError_Generic(lambda);
    }

private:


    template <typename T>
    std::vector<int> _BuildIntVector(T t)
    {
        return {t};
    }

    template<typename T, typename... Args>
    std::vector<int> _BuildIntVector(T t, Args... args) // recursive variadic function
    {
        std::vector<int> vec = _BuildIntVector(args...);
        vec.insert(vec.begin(), t);
        return vec;
    }
};


#endif // MACE_WRAPPER_H
