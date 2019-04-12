#ifndef SPATIAL_TAKEOFF_H
#define SPATIAL_TAKEOFF_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "mace.h"

#include "abstract_spatial_action.h"

#include "data_generic_command_item/command_item_type.h"

namespace CommandItem {

class SpatialTakeoff : public AbstractSpatialAction
{

public:
    SpatialTakeoff();
    SpatialTakeoff(const SpatialTakeoff &obj);
    SpatialTakeoff(const int &systemOrigin, const int &systemTarget = 0);

public:

    //!
    //! \brief getCommandType returns the type of the object that this command type is.
    //! \return Data::CommandType resolving the type of command this object is.
    //!
    COMMANDITEM getCommandType() const override;

    //!
    //! \brief getDescription
    //! \return string describing the command item. This may be useful for setting up options in a
    //! GUI or somewhere a display needs to interface and decisions have to be made describing what
    //! would happen when issuing such a command.
    //!
    std::string getDescription() const override;

    //!
    //! \brief hasSpatialInfluence returns a boolean reflecting whether or not the commandItem has
    //! a direct influence over a vehicles position. This is useful for determining flight times,
    //! position elements, or rendering objects on a GUI.
    //! \return false if the command does not have an affect over the vehicles position directly.
    //! For example, change speed has no influence over a vehicles position.
    //!
    bool hasSpatialInfluence() const override;

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
    void operator = (const SpatialTakeoff &rhs)
    {
        AbstractSpatialAction::operator =(rhs);
    }

    bool operator == (const SpatialTakeoff &rhs) {
        if(!AbstractSpatialAction::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialTakeoff &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const SpatialTakeoff& t);
};

} //end of namespace MissionItem

#endif // SPATIAL_TAKEOFF_H
