// MESSAGE AUCTION_TASK_DESCRIPTOR support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_TASK_DESCRIPTOR message
 *
 * Request for a task descriptor
 */
struct AUCTION_TASK_DESCRIPTOR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10050;
    static constexpr size_t LENGTH = 47;
    static constexpr size_t MIN_LENGTH = 47;
    static constexpr uint8_t CRC_EXTRA = 128;
    static constexpr auto NAME = "AUCTION_TASK_DESCRIPTOR";


    uint64_t sentFrom; /*< ID of agent the descriptor is being requested from */
    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    double taskGenTime; /*< Task generation time */
    uint8_t type; /*< Task type */
    uint32_t penalty; /*< Time penalty */
    double reqStart; /*< Required Start */
    double reqEnd; /*< Required End */
    uint8_t numParts; /*< Number of messages to be sent to receive the rest of the descriptor */


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
        ss << "  taskGenTime: " << taskGenTime << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  penalty: " << penalty << std::endl;
        ss << "  reqStart: " << reqStart << std::endl;
        ss << "  reqEnd: " << reqEnd << std::endl;
        ss << "  numParts: " << +numParts << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << sentFrom;                      // offset: 0
        map << creatorID;                     // offset: 8
        map << taskGenTime;                   // offset: 16
        map << reqStart;                      // offset: 24
        map << reqEnd;                        // offset: 32
        map << penalty;                       // offset: 40
        map << taskID;                        // offset: 44
        map << type;                          // offset: 45
        map << numParts;                      // offset: 46
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> sentFrom;                      // offset: 0
        map >> creatorID;                     // offset: 8
        map >> taskGenTime;                   // offset: 16
        map >> reqStart;                      // offset: 24
        map >> reqEnd;                        // offset: 32
        map >> penalty;                       // offset: 40
        map >> taskID;                        // offset: 44
        map >> type;                          // offset: 45
        map >> numParts;                      // offset: 46
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
