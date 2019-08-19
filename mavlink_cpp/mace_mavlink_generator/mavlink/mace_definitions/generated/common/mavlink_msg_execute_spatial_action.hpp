// MESSAGE EXECUTE_SPATIAL_ACTION support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief EXECUTE_SPATIAL_ACTION message
 *
 * Send a spatial command telling the vehicle to manuever based on the command type.
 */
struct EXECUTE_SPATIAL_ACTION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 35;
    static constexpr size_t LENGTH = 36;
    static constexpr size_t MIN_LENGTH = 36;
    static constexpr uint8_t CRC_EXTRA = 48;
    static constexpr auto NAME = "EXECUTE_SPATIAL_ACTION";


    uint8_t target_system; /*< System which should execute the command */
    uint8_t target_component; /*< Component which should execute the command, 0 for all components */
    uint16_t action; /*< Command ID, as defined by MAV_CMD enum. */
    uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h */
    uint8_t dimension; /*< How many dimensions the position object truly is captured in. */
    uint16_t mask; /*< Mask indicating the invalid dimensions of the position object. 1's indicate a dimesion is invalid. */
    float param1; /*< Parameter 1, as defined by MAV_CMD enum. */
    float param2; /*< Parameter 2, as defined by MAV_CMD enum. */
    float param3; /*< Parameter 3, as defined by MAV_CMD enum. */
    float param4; /*< Parameter 4, as defined by MAV_CMD enum. */
    float param5; /*< Parameter 5, as defined by MAV_CMD enum. */
    float param6; /*< Parameter 6, as defined by MAV_CMD enum. */
    float param7; /*< Parameter 7, as defined by MAV_CMD enum. */


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
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  action: " << action << std::endl;
        ss << "  frame: " << +frame << std::endl;
        ss << "  dimension: " << +dimension << std::endl;
        ss << "  mask: " << mask << std::endl;
        ss << "  param1: " << param1 << std::endl;
        ss << "  param2: " << param2 << std::endl;
        ss << "  param3: " << param3 << std::endl;
        ss << "  param4: " << param4 << std::endl;
        ss << "  param5: " << param5 << std::endl;
        ss << "  param6: " << param6 << std::endl;
        ss << "  param7: " << param7 << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << param1;                        // offset: 0
        map << param2;                        // offset: 4
        map << param3;                        // offset: 8
        map << param4;                        // offset: 12
        map << param5;                        // offset: 16
        map << param6;                        // offset: 20
        map << param7;                        // offset: 24
        map << action;                        // offset: 28
        map << mask;                          // offset: 30
        map << target_system;                 // offset: 32
        map << target_component;              // offset: 33
        map << frame;                         // offset: 34
        map << dimension;                     // offset: 35
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> param1;                        // offset: 0
        map >> param2;                        // offset: 4
        map >> param3;                        // offset: 8
        map >> param4;                        // offset: 12
        map >> param5;                        // offset: 16
        map >> param6;                        // offset: 20
        map >> param7;                        // offset: 24
        map >> action;                        // offset: 28
        map >> mask;                          // offset: 30
        map >> target_system;                 // offset: 32
        map >> target_component;              // offset: 33
        map >> frame;                         // offset: 34
        map >> dimension;                     // offset: 35
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
