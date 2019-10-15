// MESSAGE AUCTION_REQUEST_TASK_DESCRIPTOR support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_REQUEST_TASK_DESCRIPTOR message
 *
 * Request for a task descriptor
 */
struct AUCTION_REQUEST_TASK_DESCRIPTOR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10007;
    static constexpr size_t LENGTH = 17;
    static constexpr size_t MIN_LENGTH = 17;
    static constexpr uint8_t CRC_EXTRA = 22;
    static constexpr auto NAME = "AUCTION_REQUEST_TASK_DESCRIPTOR";


    uint64_t requestFrom; /*< ID of agent the task is being requested from */
    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */


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
        ss << "  requestFrom: " << requestFrom << std::endl;
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << +taskID << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << requestFrom;                   // offset: 0
        map << creatorID;                     // offset: 8
        map << taskID;                        // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> requestFrom;                   // offset: 0
        map >> creatorID;                     // offset: 8
        map >> taskID;                        // offset: 16
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
