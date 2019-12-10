// MESSAGE AUCTION_BID_DESCRIPTOR support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_BID_DESCRIPTOR message
 *
 * Message to rebroadcast a bid descriptor
 */
struct AUCTION_BID_DESCRIPTOR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10011;
    static constexpr size_t LENGTH = 76;
    static constexpr size_t MIN_LENGTH = 76;
    static constexpr uint8_t CRC_EXTRA = 35;
    static constexpr auto NAME = "AUCTION_BID_DESCRIPTOR";


    uint64_t agentID; /*< Agent ID */
    double bidGenTime; /*< Bid generation time */
    double utility; /*< Utility */
    double work; /*< Work */
    double cost; /*< Cost */
    double reward; /*< Reward */
    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    double taskGenTime; /*< Task generation time */
    uint8_t type; /*< Task type */
    uint8_t priority; /*< Priority */
    uint8_t valid; /*< Validity */
    double rebroadcastTime; /*< Time this descriptor was rebroadcast */


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
        ss << "  agentID: " << agentID << std::endl;
        ss << "  bidGenTime: " << bidGenTime << std::endl;
        ss << "  utility: " << utility << std::endl;
        ss << "  work: " << work << std::endl;
        ss << "  cost: " << cost << std::endl;
        ss << "  reward: " << reward << std::endl;
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << +taskID << std::endl;
        ss << "  taskGenTime: " << taskGenTime << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  priority: " << +priority << std::endl;
        ss << "  valid: " << +valid << std::endl;
        ss << "  rebroadcastTime: " << rebroadcastTime << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << agentID;                       // offset: 0
        map << bidGenTime;                    // offset: 8
        map << utility;                       // offset: 16
        map << work;                          // offset: 24
        map << cost;                          // offset: 32
        map << reward;                        // offset: 40
        map << creatorID;                     // offset: 48
        map << taskGenTime;                   // offset: 56
        map << rebroadcastTime;               // offset: 64
        map << taskID;                        // offset: 72
        map << type;                          // offset: 73
        map << priority;                      // offset: 74
        map << valid;                         // offset: 75
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> agentID;                       // offset: 0
        map >> bidGenTime;                    // offset: 8
        map >> utility;                       // offset: 16
        map >> work;                          // offset: 24
        map >> cost;                          // offset: 32
        map >> reward;                        // offset: 40
        map >> creatorID;                     // offset: 48
        map >> taskGenTime;                   // offset: 56
        map >> rebroadcastTime;               // offset: 64
        map >> taskID;                        // offset: 72
        map >> type;                          // offset: 73
        map >> priority;                      // offset: 74
        map >> valid;                         // offset: 75
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
