#ifndef SPATIAL_RTL_H
#define SPATIAL_RTL_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "abstract_spatial_action.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

namespace command_item {

class SpatialRTL : public AbstractSpatialAction
{

public:
    SpatialRTL();
    SpatialRTL(const SpatialRTL &obj);
    SpatialRTL(const unsigned int &originatingSystem, const unsigned int &systemTarget = 0);

public:

    //!
    //! \brief getCommandType returns the type of the object that this command type is.
    //! \return Data::CommandType resolving the type of command this object is.
    //!
    MAV_CMD getCommandType() const override;

    //!
    //! \brief getDescription
    //! \return string describing the command item. This may be useful for setting up options in a
    //! GUI or somewhere a display needs to interface and decisions have to be made describing what
    //! would happen when issuing such a command.
    //!
    std::string getDescription() const override;

    //!
    //! \brief getClone
    //! \return
    //!
    std::shared_ptr<AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractCommandItem> &command) const override;

public:
    void operator = (const SpatialRTL &rhs)
    {
        AbstractSpatialAction::operator =(rhs);
    }

    bool operator == (const SpatialRTL &rhs) {
        if(!AbstractSpatialAction::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialRTL &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printSpatialCMDInfo() const override;

};

} //end of namespace MissionItem

#endif // SPATIAL_RTL_H
