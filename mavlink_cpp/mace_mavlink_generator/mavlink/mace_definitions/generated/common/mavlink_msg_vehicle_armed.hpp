// MESSAGE VEHICLE_ARMED support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief VEHICLE_ARMED message
 *
 * A generic boolean expression indicating whether or not the vehicle is armed and ready to move about in its space.
 */
struct VEHICLE_ARMED : mavlink::Message {
    static constexpr msgid_t MSG_ID = 2;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 34;
    static constexpr auto NAME = "VEHICLE_ARMED";


    uint8_t vehicle_armed; /*< Boolean describing whether(T=1) or not(F=0) the vehicle is armed. */


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
        ss << "  vehicle_armed: " << +vehicle_armed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << vehicle_armed;                 // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> vehicle_armed;                 // offset: 0
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
