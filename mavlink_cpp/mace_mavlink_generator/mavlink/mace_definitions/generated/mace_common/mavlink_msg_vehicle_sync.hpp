// MESSAGE VEHICLE_SYNC support class

#pragma once

namespace mavlink {
namespace mace_common {
namespace msg {

/**
 * @brief VEHICLE_SYNC message
 *
 * This initiates a sync request to enable the receiving mace instance to begin dumping relevant data needed to reconstruct vehicles current state.
 */
struct VEHICLE_SYNC : mavlink::Message {
    static constexpr msgid_t MSG_ID = 50;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 178;
    static constexpr auto NAME = "VEHICLE_SYNC";


    uint8_t target_system; /*< System ID */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target_system;                 // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target_system;                 // offset: 0
    }
};

} // namespace msg
} // namespace mace_common
} // namespace mavlink
