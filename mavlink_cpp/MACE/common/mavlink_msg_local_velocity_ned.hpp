// MESSAGE LOCAL_VELOCITY_NED support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief LOCAL_VELOCITY_NED message
 *
 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
 */
struct LOCAL_VELOCITY_NED : mavlink::Message {
    static constexpr msgid_t MSG_ID = 21;
    static constexpr size_t LENGTH = 16;
    static constexpr size_t MIN_LENGTH = 16;
    static constexpr uint8_t CRC_EXTRA = 62;
    static constexpr auto NAME = "LOCAL_VELOCITY_NED";


    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot) */
    float vx; /*< X Speed */
    float vy; /*< Y Speed */
    float vz; /*< Z Speed */


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
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  vx: " << vx << std::endl;
        ss << "  vy: " << vy << std::endl;
        ss << "  vz: " << vz << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << vx;                            // offset: 4
        map << vy;                            // offset: 8
        map << vz;                            // offset: 12
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> vx;                            // offset: 4
        map >> vy;                            // offset: 8
        map >> vz;                            // offset: 12
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
