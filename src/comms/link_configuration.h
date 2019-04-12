#ifndef I_LINK_CONFIGURATION_H
#define I_LINK_CONFIGURATION_H

#include <string>

namespace Comms
{

class LinkConfiguration
{
public:

    virtual void copyFrom(LinkConfiguration* source)
    {
        Q_ASSERT(source != NULL);
        _name       = source->name();
        _dynamic    = source->isDynamic();
        _autoConnect= source->isAutoConnect();
    }

    const std::string   name(void)  { return _name; }

    void setName(const std::string &name)
    {
        _name = name;
    }


    /*!
     *
     * Is this a dynamic configuration? (non persistent)
     * @return True if this is an automatically added configuration.
     */
    bool isDynamic() { return _dynamic; }

    /*!
     *
     * Is this an Auto Connect configuration?
     * @return True if this is an Auto Connect configuration (connects automatically at boot time).
     */
    bool isAutoConnect() { return _autoConnect; }

    /*!
     * Set if this is this a dynamic configuration. (decided at runtime)
    */
    void setDynamic(bool dynamic = true) { _dynamic = dynamic; }

    /*!
     * Set if this is this an Auto Connect configuration.
    */
    void setAutoConnect(bool autoc = true) { _autoConnect = autoc; }



private:

    std::string _name;
    bool    _dynamic;       ///< A connection added automatically and not persistent (unless it's edited).
    bool    _autoConnect;   ///< This connection is started automatically at boot
};

} // END Comms

#endif // I_LINK_CONFIGURATION_H
