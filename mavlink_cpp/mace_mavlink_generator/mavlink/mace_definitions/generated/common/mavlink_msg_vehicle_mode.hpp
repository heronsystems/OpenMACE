// MESSAGE VEHICLE_MODE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief VEHICLE_MODE message
 *
 * The generic string data type containing the vehicle mode of the current system. Char messages are not easy on bandwidth to transmit and therefore should only be done when necessary indicated via a mode change.
 */
struct VEHICLE_MODE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 204;
    static constexpr auto NAME = "VEHICLE_MODE";


    std::array<char, 10> vehicle_mode; /*< The flight mode in its string form. */


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
        ss << "  vehicle_mode: \"" << to_string(vehicle_mode) << "\"" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << vehicle_mode;                  // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> vehicle_mode;                  // offset: 0
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
