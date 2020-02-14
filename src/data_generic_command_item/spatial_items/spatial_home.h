#ifndef SPATIAL_HOME_H
#define SPATIAL_HOME_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include "mace.h"

#include "abstract_spatial_action.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

using namespace mace;

namespace command_item {

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
    SpatialHome(const unsigned int &systemOrigin, const unsigned int &systemTarget = 0);

public:
    //!
    //! \brief getCommandType returns the type of the object that this command type is.
    //! \return Data::CommandType resolving the type of command this object is.
    //!
    COMMANDTYPE getCommandType() const override;

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
    //!
    //! \brief operator =
    //! \param rhs
    //!
    SpatialHome& operator = (const SpatialHome &rhs)
    {
        AbstractSpatialAction::operator =(rhs);
        this->setPosToCurrent = rhs.setPosToCurrent;
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
        if(this->setPosToCurrent != rhs.setPosToCurrent)
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

public:
    bool shouldSetToCurrent() const
    {
        return this->setPosToCurrent;
    }

    void setToCurrent(const bool &value)
    {
        this->setPosToCurrent = value;
    }

private:
    bool setPosToCurrent = false;
};

} //end of namespace command_item

#endif // SPATIAL_HOME_H
