// MESSAGE BATTERY_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief BATTERY_STATUS message
 *
 * Battery information
 */
struct BATTERY_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 3;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 227;
    static constexpr auto NAME = "BATTERY_STATUS";


    uint8_t id; /*< Battery ID */
    uint8_t battery_function; /*< Function of the battery */
    uint8_t type; /*< Type (chemistry) of the battery */
    int16_t temperature; /*< Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature. */
    uint16_t voltage_battery; /*< Battery voltage, in millivolts (1 = 1 millivolt) */
    int16_t current_battery; /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current */
    int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  id: " << +id << std::endl;
        ss << "  battery_function: " << +battery_function << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  temperature: " << temperature << std::endl;
        ss << "  voltage_battery: " << voltage_battery << std::endl;
        ss << "  current_battery: " << current_battery << std::endl;
        ss << "  battery_remaining: " << +battery_remaining << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << temperature;                   // offset: 0
        map << voltage_battery;               // offset: 2
        map << current_battery;               // offset: 4
        map << id;                            // offset: 6
        map << battery_function;              // offset: 7
        map << type;                          // offset: 8
        map << battery_remaining;             // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> temperature;                   // offset: 0
        map >> voltage_battery;               // offset: 2
        map >> current_battery;               // offset: 4
        map >> id;                            // offset: 6
        map >> battery_function;              // offset: 7
        map >> type;                          // offset: 8
        map >> battery_remaining;             // offset: 9
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
