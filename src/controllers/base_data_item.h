#ifndef BASE_DATA_ITEM_H
#define BASE_DATA_ITEM_H

#include <vector>
#include <functional>
#include <memory>

#include <unordered_map>

#include "common/optional_parameter.h"
#include "mace_core/module_characteristics.h"

namespace Controllers {

//!
//! \brief An object to be inherited by a controller that defines data I/O for that controller
//!
//! Methods in this object serve to:
//!   1) Deliver data up upon download by controller.
//!   2) Fetch data to be uploaded by controller.
//!
//! To that end three lambdas exists in this object to acomplish those goals
//! - DataReceived : Notifies a resourse that data has been received.
//! - FetchDataFromKey : Given a Key, fetch all data elements that match that key on a given resource.
//! - FetchAll : Fetch all key/values pairs on a given resource on the instance.
//!
//! Only the lambdas that are going to be used need to be set.
//! However if a lambda isn't set and is used, and exception will be thrown.
//!
//! \template Key Object that distinguishes the data on the module of origin
//! \template Type Datatype to be recevied/fetched
//!
template< typename Key, typename Type>
class DataItem
{
public:

    typedef std::vector<std::tuple<Key, Type>> FetchKeyReturn;
    typedef std::vector<std::tuple<MaceCore::ModuleCharacteristic, FetchKeyReturn>> FetchModuleReturn;

private:

    std::unordered_map<void*, std::function<void(const Key &, const Type &)>> m_lambda_DataRecieved;
    OptionalParameter<std::function<FetchKeyReturn(const OptionalParameter<Key> &)>> m_lambda_FetchDataFromKey;
    OptionalParameter<std::function<FetchModuleReturn(const OptionalParameter<MaceCore::ModuleCharacteristic> &)>> m_lambda_FetchAll;
public:

    virtual ~DataItem() = default;

    //!
    //! \brief Set the action to perform when data is received by the controller
    //! \param lambda Action to perform.
    //!
    void setLambda_DataReceived(const std::function<void(const Key &, const Type &)> &lambda){
        m_lambda_DataRecieved.insert({0, lambda});
    }


    //!
    //! \brief Set the action to perform when a controller requests for all data pertaining to a specific Key
    //! \param lambda Action to perform
    //!
    void setLambda_FetchDataFromKey(const std::function<FetchKeyReturn(const OptionalParameter<Key> &)> &lambda){
        m_lambda_FetchDataFromKey = lambda;
    }


    //!
    //! \brief Set the action to perform when a controller requests for all data on the instance
    //! \param lambda Action to perform
    //!
    void setLambda_FetchAll(const std::function<FetchModuleReturn(const OptionalParameter<MaceCore::ModuleCharacteristic> &)> &lambda)
    {
        m_lambda_FetchAll = lambda;
    }

public:


    void onDataReceived(const Key &key, const Type &data){

        for(auto it = m_lambda_DataRecieved.cbegin() ; it != m_lambda_DataRecieved.cend() ; ++it)
        {
            it->second(key, data);
        }
    }

    void FetchDataFromKey(const OptionalParameter<Key> &key, FetchKeyReturn &data) const
    {
        if(m_lambda_FetchDataFromKey.IsSet() == false) {
            throw std::runtime_error("FetchKey Lambda not set!");
        }

        data = m_lambda_FetchDataFromKey()(key);
    }

    void FetchFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &target, FetchModuleReturn &data) const
    {
        if(m_lambda_FetchAll.IsSet() == false) {
            throw std::runtime_error("FetchFromModule Lambda not set!");
        }

        data = m_lambda_FetchAll()(target);
        return;
    }
};

}

#endif // BASE_DATA_ITEM_H
