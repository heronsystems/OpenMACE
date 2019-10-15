// MESSAGE ATTITUDE_RATES support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief ATTITUDE_RATES message
 *
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
 */
struct ATTITUDE_RATES : mavlink::Message {
    static constexpr msgid_t MSG_ID = 17;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 239;
    static constexpr auto NAME = "ATTITUDE_RATES";


    float rollspeed; /*< Roll angular speed (rad/s) */
    float pitchspeed; /*< Pitch angular speed (rad/s) */
    float yawspeed; /*< Yaw angular speed (rad/s) */


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
        ss << "  rollspeed: " << rollspeed << std::endl;
        ss << "  pitchspeed: " << pitchspeed << std::endl;
        ss << "  yawspeed: " << yawspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << rollspeed;                     // offset: 0
        map << pitchspeed;                    // offset: 4
        map << yawspeed;                      // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> rollspeed;                     // offset: 0
        map >> pitchspeed;                    // offset: 4
        map >> yawspeed;                      // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
