// MESSAGE ATTITUDE_STATE_FULL support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief ATTITUDE_STATE_FULL message
 *
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
 */
struct ATTITUDE_STATE_FULL : mavlink::Message {
    static constexpr msgid_t MSG_ID = 18;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 251;
    static constexpr auto NAME = "ATTITUDE_STATE_FULL";


    float roll; /*< Roll angle (rad, -pi..+pi) */
    float pitch; /*< Pitch angle (rad, -pi..+pi) */
    float yaw; /*< Yaw angle (rad, -pi..+pi) */
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
        ss << "  roll: " << roll << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  rollspeed: " << rollspeed << std::endl;
        ss << "  pitchspeed: " << pitchspeed << std::endl;
        ss << "  yawspeed: " << yawspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << roll;                          // offset: 0
        map << pitch;                         // offset: 4
        map << yaw;                           // offset: 8
        map << rollspeed;                     // offset: 12
        map << pitchspeed;                    // offset: 16
        map << yawspeed;                      // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> roll;                          // offset: 0
        map >> pitch;                         // offset: 4
        map >> yaw;                           // offset: 8
        map >> rollspeed;                     // offset: 12
        map >> pitchspeed;                    // offset: 16
        map >> yawspeed;                      // offset: 20
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
