/** @file
 *	@brief MAVLink comm protocol generated from mace_common.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace mace_common {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 103> MESSAGE_ENTRIES {{ {0, 39, 7, 0, 0, 0}, {1, 204, 10, 0, 0, 0}, {2, 34, 1, 0, 0, 0}, {3, 227, 10, 0, 0, 0}, {4, 137, 12, 0, 0, 0}, {5, 237, 14, 3, 12, 13}, {6, 217, 28, 1, 0, 0}, {7, 104, 3, 0, 0, 0}, {8, 119, 32, 0, 0, 0}, {9, 214, 20, 3, 2, 3}, {10, 159, 2, 3, 0, 1}, {11, 220, 25, 0, 0, 0}, {12, 168, 23, 3, 4, 5}, {13, 24, 30, 0, 0, 0}, {14, 23, 101, 0, 0, 0}, {15, 115, 14, 0, 0, 0}, {16, 61, 12, 0, 0, 0}, {17, 239, 12, 0, 0, 0}, {18, 251, 24, 0, 0, 0}, {19, 246, 32, 0, 0, 0}, {20, 121, 16, 0, 0, 0}, {21, 62, 16, 0, 0, 0}, {22, 57, 28, 0, 0, 0}, {23, 187, 22, 0, 0, 0}, {24, 245, 10, 0, 0, 0}, {25, 128, 28, 0, 0, 0}, {26, 41, 13, 1, 12, 0}, {27, 39, 12, 0, 0, 0}, {28, 20, 20, 0, 0, 0}, {29, 158, 35, 3, 30, 31}, {30, 152, 33, 3, 30, 31}, {31, 177, 9, 3, 6, 7}, {32, 143, 3, 0, 0, 0}, {33, 129, 21, 1, 0, 0}, {34, 1, 1, 0, 0, 0}, {35, 48, 36, 3, 32, 33}, {36, 108, 1, 0, 0, 0}, {50, 178, 1, 1, 0, 0}, {51, 189, 21, 3, 16, 17}, {100, 132, 5, 0, 0, 0}, {101, 18, 7, 1, 2, 0}, {102, 87, 7, 0, 0, 0}, {103, 135, 3, 0, 0, 0}, {104, 84, 5, 0, 0, 0}, {105, 165, 8, 1, 2, 0}, {106, 3, 8, 1, 2, 0}, {107, 49, 41, 1, 32, 0}, {108, 4, 7, 3, 4, 5}, {109, 168, 7, 3, 4, 5}, {110, 224, 4, 0, 0, 0}, {111, 156, 5, 0, 0, 0}, {112, 47, 7, 0, 0, 0}, {113, 54, 7, 0, 0, 0}, {114, 34, 4, 1, 0, 0}, {115, 55, 5, 0, 0, 0}, {116, 36, 1, 1, 0, 0}, {117, 149, 53, 0, 0, 0}, {118, 85, 53, 1, 52, 0}, {119, 21, 2, 1, 0, 0}, {120, 12, 18, 0, 0, 0}, {130, 52, 6, 0, 0, 0}, {131, 220, 4, 0, 0, 0}, {132, 171, 3, 0, 0, 0}, {133, 122, 5, 0, 0, 0}, {134, 138, 5, 0, 0, 0}, {135, 66, 18, 0, 0, 0}, {200, 185, 9, 0, 0, 0}, {201, 34, 16, 0, 0, 0}, {202, 203, 6, 0, 0, 0}, {203, 85, 14, 0, 0, 0}, {204, 47, 32, 0, 0, 0}, {244, 95, 6, 0, 0, 0}, {245, 130, 2, 0, 0, 0}, {246, 184, 38, 0, 0, 0}, {247, 81, 19, 0, 0, 0}, {253, 83, 51, 0, 0, 0}, {259, 122, 86, 0, 0, 0}, {260, 8, 28, 0, 0, 0}, {262, 69, 31, 0, 0, 0}, {263, 133, 255, 0, 0, 0}, {264, 49, 28, 0, 0, 0}, {10000, 72, 17, 0, 0, 0}, {10001, 74, 84, 0, 0, 0}, {10002, 175, 17, 0, 0, 0}, {10003, 174, 16, 0, 0, 0}, {10004, 150, 16, 0, 0, 0}, {10005, 239, 21, 0, 0, 0}, {10006, 65, 9, 0, 0, 0}, {10007, 22, 17, 0, 0, 0}, {10008, 200, 9, 0, 0, 0}, {10009, 91, 82, 0, 0, 0}, {10010, 201, 10, 0, 0, 0}, {10011, 35, 76, 0, 0, 0}, {10012, 97, 25, 0, 0, 0}, {10013, 229, 25, 0, 0, 0}, {10014, 231, 25, 0, 0, 0}, {10050, 128, 47, 0, 0, 0}, {10051, 226, 17, 0, 0, 0}, {10052, 214, 17, 0, 0, 0}, {10053, 180, 18, 0, 0, 0}, {10054, 133, 48, 0, 0, 0}, {10055, 54, 43, 0, 0, 0}, {10056, 95, 34, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS


/** @brief Type of mission items being requested/sent in mission protocol. */
enum class POINT_DISCOVERY : uint8_t
{
    EXISTING=0, /* The point reported is an existing point in the list relative to the original assignment. | */
    NEW=1, /* The point reported is newly discovered relative to the list initially recieved. | */
};

//! POINT_DISCOVERY ENUM_END
constexpr auto POINT_DISCOVERY_ENUM_END = 2;

/** @brief Type of mission items being requested/sent in mission protocol. */
enum class STRESS_THRESHOLD : uint8_t
{
    STRESS_MAX=0, /* State of the stress point has been confirmed at a maximum. | */
    AMBIGUOUS=1, /* State of the stress point has yet to be confirmed. | */
    STRESS_MIN=2, /* State of the stress point has been confirmed at a minimum. | */
};

//! STRESS_THRESHOLD ENUM_END
constexpr auto STRESS_THRESHOLD_ENUM_END = 3;


} // namespace mace_common
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_vehicle_sync.hpp"
#include "./mavlink_msg_roi_ag.hpp"

// base include
#include "../common/common.hpp"
#include "../mission/mission.hpp"
#include "../boundary/boundary.hpp"
#include "../auctioneer/auctioneer.hpp"
