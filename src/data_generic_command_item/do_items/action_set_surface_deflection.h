#ifndef ACTION_SET_SURFACE_DEFLECTION_H
#define ACTION_SET_SURFACE_DEFLECTION_H

#include "common/common.h"
#include "common/class_forward.h"

#include "data_generic_command_item/spatial_items/spatial_components.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

#include "data/jsonconverter.h"

namespace command_item {

class Action_SetSurfaceDeflection : public AbstractCommandItem, public JSONConverter
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
    MAV_CMD getCommandType() const override;

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

    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */


public:
    void operator = (const Action_SetSurfaceDeflection &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        _surfaceDeflection = rhs._surfaceDeflection;
        _time = rhs._time;
    }

    bool operator == (const Action_SetSurfaceDeflection &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->_surfaceDeflection != rhs._surfaceDeflection){
            return false;
        }
        if(_time != rhs._time)
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

    void setTime();

    friend std::ostream &operator<<(std::ostream &out, const Action_SetSurfaceDeflection &obj)
    {
        UNUSED(obj);
        //        out<<"Command Change Mode( Mode: "<<obj.vehicleMode<<")";
        return out;
    }
public:
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType = "") const override;

    virtual void fromJSON(const QJsonDocument &inputJSON);

    virtual std::string toCSV(const std::string &delimiter) const;

public:
    struct Deflection
    {
        double _aileron = 0.0;
        double _elevator = 0.0;
        double _rudder = 0.0;
        double _throttle = 0.0;

        Deflection() = default;

        Deflection(const double &aileron, const double &elevator, const double &rudder, const double &throttle)
        {
            _aileron = aileron;
            _elevator = elevator;
            _rudder = rudder;
            _throttle = throttle;
        }

        void operator = (const Deflection &rhs)
        {
            _aileron = rhs._aileron;
            _elevator = rhs._elevator;
            _rudder = rhs._rudder;
            _throttle = rhs._throttle;
        }

        bool operator == (const Deflection &rhs) {
            if(fabs(this->_aileron - rhs._aileron) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_elevator - rhs._elevator) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_rudder - rhs._rudder) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_throttle - rhs._throttle) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            return true;
        }

        bool operator != (const Deflection &rhs) {
            return !(*this == rhs);
        }

    };

public:
    Data::EnvironmentTime _time;
    Deflection _surfaceDeflection;

};

} //end of namespace command_item

#endif // ACTION_SETSURFACEDEFLECTION_H
