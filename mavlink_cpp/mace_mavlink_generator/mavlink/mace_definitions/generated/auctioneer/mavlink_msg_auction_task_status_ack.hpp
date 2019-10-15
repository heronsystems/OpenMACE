// MESSAGE AUCTION_TASK_STATUS_ACK support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_TASK_STATUS_ACK message
 *
 * Acknowledgment of a task complete message
 */
struct AUCTION_TASK_STATUS_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10010;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 201;
    static constexpr auto NAME = "AUCTION_TASK_STATUS_ACK";


    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    uint8_t type; /*< Status update type */


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
        ss << "  taskID: " << +taskID << std::endl;
        ss << "  type: " << +type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << creatorID;                     // offset: 0
        map << taskID;                        // offset: 8
        map << type;                          // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> creatorID;                     // offset: 0
        map >> taskID;                        // offset: 8
        map >> type;                          // offset: 9
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
