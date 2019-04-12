#ifndef MODULEFACTORY_H
#define MODULEFACTORY_H

#include <unordered_map>
#include <functional>
#include <list>

#include "abstract_module_base.h"

#include <iostream>

namespace MaceCore
{



class ModuleFactory
{
public:


    ModuleFactory()
    {
    }

    //!
    //! \brief RegisterFactory Register module factory
    //! \param type Module type
    //! \param moduleName Module name
    //! \param createExpression Method of creation (e.g. std::make_shared<T>())
    //! \return True if successful
    //!
    bool RegisterFactory(const ModuleClasses &type, const std::string &moduleName, std::function<std::shared_ptr<ModuleBase>()> createExpression)
    {
        if(_factories.find(type) == _factories.cend())
            _factories.insert({type, std::unordered_map<std::string, std::function<std::shared_ptr<ModuleBase>()>>()});

        _factories.at(type).insert({moduleName, createExpression});

        return true;
    }

    //!
    //! \brief Create Create module
    //! \param type Module type
    //! \param moduleName Module name
    //! \return Module pointer
    //!
    std::shared_ptr<ModuleBase> Create(const ModuleClasses &type, const std::string &moduleName) const
    {

        if(_factories.find(type) == _factories.cend())
            throw std::runtime_error("Provided module class has no entires in factory");


        if(_factories.at(type).find(moduleName) == _factories.at(type).cend())
            throw std::runtime_error("Provided Module name "+ moduleName +" does not exists for the given class");

        return _factories.at(type).at(moduleName)();
    }

    //!
    //! \brief GetTypes Get list of modules of a certain type
    //! \param type Module type
    //! \return List of module names with the given type
    //!
    std::list<std::string> GetTypes(const ModuleClasses &type) const
    {
        std::list<std::string> result;
        for(auto& item: _factories.at(type))
        {
            result.push_back(item.first);
        }
        return result;
    }


private:
    std::map<ModuleClasses, std::unordered_map<std::string, std::function<std::shared_ptr<ModuleBase>()>>> _factories;
};


} // END MaceCore Namespace

#endif // MODULEFACTORY_H
