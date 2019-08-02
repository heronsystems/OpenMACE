#ifndef SPATIAL_HOME_H
#define SPATIAL_HOME_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "mace.h"

#include "abstract_spatial_action.h"

#include "data_generic_command_item/command_item_type.h"

using namespace mace;

namespace CommandItem {

//!
//! \brief The SpatialHome class
//!
class SpatialHome : public AbstractSpatialAction
{
public:
    //!
    //! \brief SpatialHome The default constructor for a SpatialHome commandItem. This will default the
    //! originating and target systems to 0. Additionally, the position will default to 0,0,0 with
    //! a global relative alt coordinate frame definition.
    //!
    SpatialHome();

    //!
    //! \brief SpatialHome
    //! \param homePosition
    //!
    SpatialHome(const mace::pose::Position* homePosition);

    ~SpatialHome() override;

    //!
    //! \brief SpatialHome A default copy constructor of a SpatialHome commandItem object.
    //! \param obj of type SpatialHome that the data shall be copied from.
    //!
    SpatialHome(const SpatialHome &copy);

    //!
    //! \brief SpatialHome An overloaded default constructor for a SpatialHome commandItem.
    //! \param systemOrigin The ID value of the system that initiated the command item.
    //! \param systemTarget The ID value of the system that is the intended recipient of the command item.
    //! A developer should be aware that this value defaults to 0 if no arguments are provided.
    //!
    SpatialHome(const int &systemOrigin, const int &systemTarget = 0);


public:
    //!
    //! \brief toMACEComms_MissionItem
    //! \param obj
    //! \return
    //!
    bool toMACEComms_MissionItem(mace_mission_item_t &obj) const;

    //!
    //! \brief toMACEComms_MSG
    //! \param obj
    //! \return
    //!
    mace_message_t toMACEComms_MSG(mace_mission_item_t &obj) const;

public:
    //!
    //! \brief fromMACEComms_CommandObject
    //! \param obj
    //! \return
    //!
    bool fromMACEComms_CommandObject(const mace_set_home_position_t &obj);

    //!
    //! \brief getMACECommsObject
    //! \return
    //!
    bool getMACECommsObject(mace_home_position_t &obj) const;

    //!
    //! \brief getMACEMsg
    //! \param systemID
    //! \param compID
    //! \param chan
    //! \return
    //!
    bool getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan, mace_message_t &msg) const;

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
    //!
    //! \brief operator =
    //! \param rhs
    //!
    SpatialHome& operator = (const SpatialHome &rhs)
    {
        AbstractSpatialAction::operator =(rhs);
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return 
    //!
    bool operator == (const SpatialHome &rhs) {
        if(!AbstractSpatialAction::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return 
    //!
    bool operator != (const SpatialHome &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printSpatialCMDInfo() const override;

    friend std::ostream& operator<<(std::ostream& os, const SpatialHome& t);
};

} //end of namespace MissionItem

#endif // SPATIAL_HOME_H
