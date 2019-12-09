// MESSAGE AUCTION_TASK_DESCRIPTOR_LOITER_DATA support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_TASK_DESCRIPTOR_LOITER_DATA message
 *
 * 
 */
struct AUCTION_TASK_DESCRIPTOR_LOITER_DATA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10054;
    static constexpr size_t LENGTH = 48;
    static constexpr size_t MIN_LENGTH = 48;
    static constexpr uint8_t CRC_EXTRA = 133;
    static constexpr auto NAME = "AUCTION_TASK_DESCRIPTOR_LOITER_DATA";


    uint64_t sentFrom; /*< ID of agent the task is being requested from */
    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    int8_t seqNum; /*< Sequence number */
    double xPos; /*< x position */
    double yPos; /*< y position */
    double zPos; /*< z position */
    uint32_t duration; /*< duration */
    uint8_t coordinateType; /*< Coordinate type. See TaskDescriptor::TaskType */
    uint8_t coordinateFrame; /*< Coordinate frame. See mace::pose::CoordinateFrame */


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
        ss << "  zPos: " << zPos << std::endl;
        ss << "  duration: " << duration << std::endl;
        ss << "  coordinateType: " << +coordinateType << std::endl;
        ss << "  coordinateFrame: " << +coordinateFrame << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << sentFrom;                      // offset: 0
        map << creatorID;                     // offset: 8
        map << xPos;                          // offset: 16
        map << yPos;                          // offset: 24
        map << zPos;                          // offset: 32
        map << duration;                      // offset: 40
        map << taskID;                        // offset: 44
        map << seqNum;                        // offset: 45
        map << coordinateType;                // offset: 46
        map << coordinateFrame;               // offset: 47
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> sentFrom;                      // offset: 0
        map >> creatorID;                     // offset: 8
        map >> xPos;                          // offset: 16
        map >> yPos;                          // offset: 24
        map >> zPos;                          // offset: 32
        map >> duration;                      // offset: 40
        map >> taskID;                        // offset: 44
        map >> seqNum;                        // offset: 45
        map >> coordinateType;                // offset: 46
        map >> coordinateFrame;               // offset: 47
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
