// MESSAGE AUCTION_BID_BUNDLE_NEXT_PART support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_BID_BUNDLE_NEXT_PART message
 *
 * Message to send the next bid bundle part
 */
struct AUCTION_BID_BUNDLE_NEXT_PART : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10001;
    static constexpr size_t LENGTH = 84;
    static constexpr size_t MIN_LENGTH = 84;
    static constexpr uint8_t CRC_EXTRA = 74;
    static constexpr auto NAME = "AUCTION_BID_BUNDLE_NEXT_PART";


    uint64_t requestFrom; /*< ID of agent requesting the bundle */
    uint64_t agentID; /*< ID of agent generating the bundle */
    double bundleGenTime; /*< Bundle generation time */
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
    int8_t seqNum; /*< Sequence number */


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
        ss << "  agentID: " << agentID << std::endl;
        ss << "  bundleGenTime: " << bundleGenTime << std::endl;
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
        ss << "  seqNum: " << +seqNum << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << requestFrom;                   // offset: 0
        map << agentID;                       // offset: 8
        map << bundleGenTime;                 // offset: 16
        map << bidGenTime;                    // offset: 24
        map << utility;                       // offset: 32
        map << work;                          // offset: 40
        map << cost;                          // offset: 48
        map << reward;                        // offset: 56
        map << creatorID;                     // offset: 64
        map << taskGenTime;                   // offset: 72
        map << taskID;                        // offset: 80
        map << type;                          // offset: 81
        map << priority;                      // offset: 82
        map << seqNum;                        // offset: 83
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> requestFrom;                   // offset: 0
        map >> agentID;                       // offset: 8
        map >> bundleGenTime;                 // offset: 16
        map >> bidGenTime;                    // offset: 24
        map >> utility;                       // offset: 32
        map >> work;                          // offset: 40
        map >> cost;                          // offset: 48
        map >> reward;                        // offset: 56
        map >> creatorID;                     // offset: 64
        map >> taskGenTime;                   // offset: 72
        map >> taskID;                        // offset: 80
        map >> type;                          // offset: 81
        map >> priority;                      // offset: 82
        map >> seqNum;                        // offset: 83
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
