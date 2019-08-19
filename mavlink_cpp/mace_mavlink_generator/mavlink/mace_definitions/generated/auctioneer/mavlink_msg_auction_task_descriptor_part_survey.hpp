// MESSAGE AUCTION_TASK_DESCRIPTOR_PART_SURVEY support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_TASK_DESCRIPTOR_PART_SURVEY message
 *
 * 
 */
struct AUCTION_TASK_DESCRIPTOR_PART_SURVEY : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10056;
    static constexpr size_t LENGTH = 34;
    static constexpr size_t MIN_LENGTH = 34;
    static constexpr uint8_t CRC_EXTRA = 95;
    static constexpr auto NAME = "AUCTION_TASK_DESCRIPTOR_PART_SURVEY";


    uint64_t sentFrom; /*< ID of agent the task is being requested from */
    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    int8_t seqNum; /*< Sequence number */
    double xPos; /*< x position */
    double yPos; /*< y position */


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
        ss << "  sentFrom: " << sentFrom << std::endl;
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << +taskID << std::endl;
        ss << "  seqNum: " << +seqNum << std::endl;
        ss << "  xPos: " << xPos << std::endl;
        ss << "  yPos: " << yPos << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << sentFrom;                      // offset: 0
        map << creatorID;                     // offset: 8
        map << xPos;                          // offset: 16
        map << yPos;                          // offset: 24
        map << taskID;                        // offset: 32
        map << seqNum;                        // offset: 33
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> sentFrom;                      // offset: 0
        map >> creatorID;                     // offset: 8
        map >> xPos;                          // offset: 16
        map >> yPos;                          // offset: 24
        map >> taskID;                        // offset: 32
        map >> seqNum;                        // offset: 33
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
