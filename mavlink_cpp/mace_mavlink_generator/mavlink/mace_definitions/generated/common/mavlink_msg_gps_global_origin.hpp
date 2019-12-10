// MESSAGE GPS_GLOBAL_ORIGIN support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief GPS_GLOBAL_ORIGIN message
 *
 * Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
 */
struct GPS_GLOBAL_ORIGIN : mavlink::Message {
    static constexpr msgid_t MSG_ID = 27;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 39;
    static constexpr auto NAME = "GPS_GLOBAL_ORIGIN";


    int32_t latitude; /*< Latitude (WGS84), in degrees * 1E7 */
    int32_t longitude; /*< Longitude (WGS84), in degrees * 1E7 */
    int32_t altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up) */


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
        ss << "  latitude: " << latitude << std::endl;
        ss << "  longitude: " << longitude << std::endl;
        ss << "  altitude: " << altitude << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << latitude;                      // offset: 0
        map << longitude;                     // offset: 4
        map << altitude;                      // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> latitude;                      // offset: 0
        map >> longitude;                     // offset: 4
        map >> altitude;                      // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
