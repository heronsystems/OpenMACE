// MESSAGE GLOBAL_VELOCITY_INT support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief GLOBAL_VELOCITY_INT message
 *
 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
               is designed as scaled integer message since the resolution of float is not sufficient.
 */
struct GLOBAL_VELOCITY_INT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 24;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 245;
    static constexpr auto NAME = "GLOBAL_VELOCITY_INT";


    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot) */
    int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100 */
    int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100 */
    int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100 */


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
        map << vy;                            // offset: 6
        map << vz;                            // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> vx;                            // offset: 4
        map >> vy;                            // offset: 6
        map >> vz;                            // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
