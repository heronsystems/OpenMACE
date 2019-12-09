// MESSAGE ROI_AG support class

#pragma once

namespace mavlink {
namespace mace_common {
namespace msg {

/**
 * @brief ROI_AG message
 *
 * Message indicating the current state that was required for visiting and sensing state.
 */
struct ROI_AG : mavlink::Message {
    static constexpr msgid_t MSG_ID = 51;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 189;
    static constexpr auto NAME = "ROI_AG";


    uint8_t target_system; /*< System ID */
    uint8_t target_component; /*< Component ID */
    uint8_t point_discovery; /*< See POINT_DISCOVERY enum */
    uint8_t stress_threshold; /*< See STRESS_THRESHOLD enum */
    float stress_value; /*< Stress value */
    uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h */
    float x; /*< PARAM5 / local: x position, global: latitude */
    float y; /*< PARAM6 / y position: global: longitude */
    float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame. */


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
        ss << "  point_discovery: " << +point_discovery << std::endl;
        ss << "  stress_threshold: " << +stress_threshold << std::endl;
        ss << "  stress_value: " << stress_value << std::endl;
        ss << "  frame: " << +frame << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << stress_value;                  // offset: 0
        map << x;                             // offset: 4
        map << y;                             // offset: 8
        map << z;                             // offset: 12
        map << target_system;                 // offset: 16
        map << target_component;              // offset: 17
        map << point_discovery;               // offset: 18
        map << stress_threshold;              // offset: 19
        map << frame;                         // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> stress_value;                  // offset: 0
        map >> x;                             // offset: 4
        map >> y;                             // offset: 8
        map >> z;                             // offset: 12
        map >> target_system;                 // offset: 16
        map >> target_component;              // offset: 17
        map >> point_discovery;               // offset: 18
        map >> stress_threshold;              // offset: 19
        map >> frame;                         // offset: 20
    }
};

} // namespace msg
} // namespace mace_common
} // namespace mavlink
