// MESSAGE ATTITUDE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief ATTITUDE message
 *
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
 */
struct ATTITUDE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 16;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 61;
    static constexpr auto NAME = "ATTITUDE";


    float roll; /*< Roll angle (rad, -pi..+pi) */
    float pitch; /*< Pitch angle (rad, -pi..+pi) */
    float yaw; /*< Yaw angle (rad, -pi..+pi) */


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
        ss << "  roll: " << roll << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  yaw: " << yaw << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << roll;                          // offset: 0
        map << pitch;                         // offset: 4
        map << yaw;                           // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> roll;                          // offset: 0
        map >> pitch;                         // offset: 4
        map >> yaw;                           // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
