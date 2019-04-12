/** @file
 *  @brief MAVLink comm protocol built from mission.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_H
#define MACE_H

#define MACE_PRIMARY_XML_IDX 2

#ifndef MACE_STX
#define MACE_STX 253
#endif

#ifndef MACE_ENDIAN
#define MACE_ENDIAN MACE_LITTLE_ENDIAN
#endif

#ifndef MACE_ALIGNED_FIELDS
#define MACE_ALIGNED_FIELDS 1
#endif

#ifndef MACE_CRC_EXTRA
#define MACE_CRC_EXTRA 1
#endif

#ifndef MACE_COMMAND_24BIT
#define MACE_COMMAND_24BIT 1
#endif

#include "version.h"
#include "mission.h"

#endif // MACE_H
