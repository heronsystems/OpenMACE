// MESSAGE AUCTION_NEW_TASK_KEY support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_NEW_TASK_KEY message
 *
 * Notification of a new task key
 */
struct AUCTION_NEW_TASK_KEY : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10005;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 239;
    static constexpr auto NAME = "AUCTION_NEW_TASK_KEY";


    uint64_t creatorID; /*< Task creator ID */
    uint32_t taskID; /*< Creator local task ID */
    double taskGenTime; /*< Task generation time */
    uint8_t type; /*< Task type */


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
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << taskID << std::endl;
        ss << "  taskGenTime: " << taskGenTime << std::endl;
        ss << "  type: " << +type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << creatorID;                     // offset: 0
        map << taskGenTime;                   // offset: 8
        map << taskID;                        // offset: 16
        map << type;                          // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> creatorID;                     // offset: 0
        map >> taskGenTime;                   // offset: 8
        map >> taskID;                        // offset: 16
        map >> type;                          // offset: 20
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
