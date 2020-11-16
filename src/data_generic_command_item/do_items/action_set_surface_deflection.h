#ifndef ACTION_SET_SURFACE_DEFLECTION_H
#define ACTION_SET_SURFACE_DEFLECTION_H

#include "common/common.h"
#include "common/class_forward.h"

#include "../spatial_items/spatial_components.h"
#include "../spatial_items/spatial_action_factory.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

namespace command_item {

class Action_SetSurfaceDeflection : public AbstractCommandItem, public Interface_CommandHelper<mace_command_long_t>
{
public:
    Action_SetSurfaceDeflection();
    Action_SetSurfaceDeflection(const Action_SetSurfaceDeflection &obj);
    Action_SetSurfaceDeflection(const unsigned int &systemOrigin, const unsigned int &systemTarget);

    ~Action_SetSurfaceDeflection() override
    {

    }

public:
    /**
     * @brief getCommandType
     * @return
     */
    COMMANDTYPE getCommandType() const override;

    /**
     * @brief getDescription
     * @return
     */
    std::string getDescription() const override;

    /**
     * @brief hasSpatialInfluence
     * @return
     */
    bool hasSpatialInfluence() const override;

    /**
     * @brief getClone
     * @return
     */
    std::shared_ptr<AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractCommandItem> &command) const override;

    /** Interface imposed via Interface_CommandItem<mace_command_short_t> */
public:
    void populateCommandItem(mace_command_long_t &obj) const override;

    void fromCommandItem(const mace_command_long_t &obj) override;

    /** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd) override;

    void generateMACEMSG_MissionItem(mace_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mace_message_t &msg) const override; //we know that you must cast to the specific type to get something explicit based on the command

    /** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

public:
    void operator = (const Action_SetSurfaceDeflection &rhs)
    {
        AbstractCommandItem::operator =(rhs);
    }

    bool operator == (const Action_SetSurfaceDeflection &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        return true;
    }

    bool operator != (const Action_SetSurfaceDeflection &rhs) {
        return !(*this == rhs);
    }
public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;


    friend std::ostream &operator<<(std::ostream &out, const Action_SetSurfaceDeflection &obj)
    {
        UNUSED(obj);
        //        out<<"Command Change Mode( Mode: "<<obj.vehicleMode<<")";
        return out;
    }

public:
    struct Deflection
    {
        double roll;
        double pitch;
        double yaw;
        double throttle;
    };

public:
    Deflection _surfaceDeflection;
};

} //end of namespace command_item

#endif // ACTION_SETSURFACEDEFLECTION_H
