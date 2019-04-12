#ifndef METADATA_RTA_H
#define METADATA_RTA_H

#include <memory>
#include "abstract_module_base.h"

namespace MaceCore
{

class Metadata_RTA
{
private:

    bool m_IsGlobal;
    int m_Specalization;

public:

    Metadata_RTA() :
        m_IsGlobal(true),
        m_Specalization(-1)
    {

    }

    void SetGlobal()
    {
        m_IsGlobal = true;
    }

    void SetSpecalization(int moduleID)
    {
        m_IsGlobal = false;
        m_Specalization = moduleID;
    }


    //!
    //! \brief Determine if the RTA module is a global instance
    //! \return True if global
    //!
    bool IsGlobal() const
    {
        return m_IsGlobal;
    }


    //!
    //! \brief Determine what this RTA module is a specalization of.
    //! \return Pointer to module this RTA is a specalization of. NULL if instance is global.
    //!
    int SpecalizationOf() const
    {
        if(m_IsGlobal)
        {
            throw std::runtime_error("Module is a global instance, It is not a specialization");
        }
        else
        {
            return m_Specalization;
        }
    }
};

} //End MaceCore Namespace

#endif // METADATA_RTA_H
